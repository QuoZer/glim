#!/usr/bin/env python3
"""Lightweight two-LiDAR merger for GLIM (no PCL deps).

Subscribes to the two Livox HAP PointCloud2 topics, transforms each into
`base_link` using the static extrinsics from scans_merger/merge_nn.launch.py,
concatenates them, and republishes as /merged_scan.

Unlike the PCL XYZI merger, this PRESERVES the per-point `timestamp` field so
GLIM can still deskew. Output fields: x, y, z, intensity, timestamp.
"""
import os
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import Header
import message_filters

# Diagnostic: GLIM_LEFT_ONLY=1 makes the node republish ONLY the left sensor
# (native, identity frame) as /merged_scan, i.e. the known-good single-LiDAR
# geometry. Use it to tell a merge-extrinsic warp apart from an IMU/degeneracy
# slope: if left-only maps flat but the 2-LiDAR merge slopes, the right cloud's
# relative pitch is the culprit, not the IMU.
LEFT_ONLY = os.environ.get('GLIM_LEFT_ONLY', '0') not in ('0', '', 'false', 'False')

# Pre-merge point reduction, applied independently to each input cloud before
# the transform+concat (so it also shrinks the transform/publish/downstream
# GLIM-ingestion cost proportionally, not just the wire size).
#   GLIM_MERGE_STRIDE : keep every Nth point (int, default 1 = off). O(1)
#                       slicing, effectively free, but the kept subset follows
#                       the sensor's native scan-line pattern rather than
#                       being spatially uniform.
#   GLIM_MERGE_VOXEL  : keep one (first-hit) point per occupied voxel of this
#                       size in meters (float, default 0.0 = off). Spatially
#                       uniform; costs ~2ms per 41k-pt HAP scan via a packed-
#                       int64-key np.unique (np.unique(axis=0) on raw float
#                       coords was measured ~13x slower -- not used). Applied
#                       after STRIDE if both are set.
GLIM_MERGE_STRIDE = int(os.environ.get('GLIM_MERGE_STRIDE', '1'))
GLIM_MERGE_VOXEL = float(os.environ.get('GLIM_MERGE_VOXEL', '0.0'))
assert GLIM_MERGE_STRIDE >= 1, 'GLIM_MERGE_STRIDE must be >= 1'
assert GLIM_MERGE_VOXEL >= 0.0, 'GLIM_MERGE_VOXEL must be >= 0'
_VOXEL_BASE = 1 << 21
_VOXEL_OFF = 1 << 20  # covers +-100m at any voxel size >= ~0.0001m


def downsample(xyz, inten, ts):
    if GLIM_MERGE_STRIDE > 1:
        xyz, inten, ts = xyz[::GLIM_MERGE_STRIDE], inten[::GLIM_MERGE_STRIDE], ts[::GLIM_MERGE_STRIDE]
    if GLIM_MERGE_VOXEL > 0.0 and xyz.shape[0]:
        ijk = np.floor(xyz / GLIM_MERGE_VOXEL).astype(np.int64) + _VOXEL_OFF
        keys = (ijk[:, 0] * _VOXEL_BASE + ijk[:, 1]) * _VOXEL_BASE + ijk[:, 2]
        _, idx = np.unique(keys, return_index=True)
        xyz, inten, ts = xyz[idx], inten[idx], ts[idx]
    return xyz, inten, ts


def ypr_matrix(yaw, pitch, roll):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


# Static poses of each sensor in base_link.
# Left: target calibration values at full precision (yaw 40.756, pitch -3.570,
# roll -0.601 deg). Right: refined 2026-07-17 by joint GICP on two static
# sections of the nora bag (parked garage t=243-256s + roadside stop t=725-750s,
# cross-validated to 0.17 deg / 7 cm between scenes; reflective-rectangle
# landmark residual 2.5 cm). Supersedes the CAD-symmetric pose
# ([0, -1.1135164607, -0.0375], ypr(-0.66, 0, 0)): the physical lidar-to-lidar
# baseline is ~2.07 m, not 2.23 m, and right roll/pitch are not zero.
_BL_LEFT  = (np.array([0.0,  1.1135164607,  0.0375]),
             ypr_matrix(0.7113264, -0.0623083, -0.0104894))
_BL_RIGHT = (np.array([0.0660688, -0.9572868, 0.0400101]),
             ypr_matrix(-0.6787543, -0.0125032, -0.0075282))

# Publish the merged cloud in the LEFT sensor frame (= the left IMU frame) so
# GLIM's T_lidar_imu is identity and the hand-authored lever arm can't inject a
# persistent IMU-prediction error. Left stays native; right is mapped
# left <- right via  T_left_right = inv(T_bl_left) * T_bl_right.
_tL, _RL = _BL_LEFT
_tR, _RR = _BL_RIGHT
EXTRINSICS = {
    'left':  (np.zeros(3), np.eye(3)),
    'right': (_RL.T @ (_tR - _tL), _RL.T @ _RR),
}
FRAME_ID = 'livox_hap_left_frame'

# Input Livox HAP point layout (point_step 26)
IN_DTYPE = np.dtype({
    'names':   ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
    'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
    'offsets': [0, 4, 8, 12, 16, 17, 18],
    'itemsize': 26,
})


def parse(msg):
    arr = np.frombuffer(msg.data, dtype=IN_DTYPE, count=msg.width * msg.height)
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
    return xyz, arr['intensity'].astype(np.float32), arr['timestamp'].astype(np.float64)


IMU_MAX_GAP_NS = int(20e6)  # right-IMU bracket must be within 20ms, else fall back to left-only


class Merger(Node):
    def __init__(self):
        super().__init__('cloud_merger_py')
        self.pub = self.create_publisher(PointCloud2, '/merged_scan', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/merged_imu', qos_profile_sensor_data)
        self.n = 0
        self.n_imu = 0
        # Ring buffer of recent right-IMU samples for interpolation -- see
        # cb_imu(). ~40 samples is ~200ms of margin at ~199Hz.
        self._right_imu_buf = deque(maxlen=40)
        if LEFT_ONLY:
            self.sub = self.create_subscription(
                PointCloud2, '/livox/lidar_10_0_0_50', self.cb_left_only,
                qos_profile_sensor_data)
            self.imu_sub = self.create_subscription(
                Imu, '/livox/imu_10_0_0_50', self.cb_imu_left_only, qos_profile_sensor_data)
            self.get_logger().info('cloud_merger_py up: LEFT-ONLY /livox/lidar_*_50 + imu_*_50 passthrough')
        else:
            s1 = message_filters.Subscriber(self, PointCloud2, '/livox/lidar_10_0_0_50',
                                            qos_profile=qos_profile_sensor_data)
            s2 = message_filters.Subscriber(self, PointCloud2, '/livox/lidar_10_0_0_51',
                                            qos_profile=qos_profile_sensor_data)
            self.sync = message_filters.ApproximateTimeSynchronizer([s1, s2], queue_size=10, slop=0.05)
            self.sync.registerCallback(self.cb)

            # IMU: left drives the fused output timing; right is buffered and
            # interpolated onto each left timestamp (see cb_imu() for why --
            # nearest-neighbor pairing at a fixed slop doesn't work here, the
            # two sensors' clocks aren't hardware-synced).
            self.imu_r_sub = self.create_subscription(
                Imu, '/livox/imu_10_0_0_51', self._buffer_right_imu, qos_profile_sensor_data)
            self.imu_l_sub = self.create_subscription(
                Imu, '/livox/imu_10_0_0_50', self.cb_imu, qos_profile_sensor_data)

            self.get_logger().info('cloud_merger_py up: /livox/lidar_*_50,_51 -> /merged_scan; '
                                    '/livox/imu_*_50,_51 -> /merged_imu (left frame)')
            if GLIM_MERGE_STRIDE > 1 or GLIM_MERGE_VOXEL > 0.0:
                self.get_logger().info(
                    f'pre-merge downsample: stride={GLIM_MERGE_STRIDE} voxel={GLIM_MERGE_VOXEL}m')

    def cb_left_only(self, c_left):
        # Left sensor is native / identity in FRAME_ID -> no transform.
        xl, il, tl = downsample(*parse(c_left))
        self.publish(xl.astype(np.float32), il, tl, c_left.header.stamp)

    def cb(self, c_left, c_right):
        xl, il, tl = downsample(*parse(c_left))
        xr, ir, tr = downsample(*parse(c_right))
        tL, RL = EXTRINSICS['left']
        tR, RR = EXTRINSICS['right']
        pl = xl @ RL.T + tL
        pr = xr @ RR.T + tR
        xyz = np.vstack([pl, pr]).astype(np.float32)
        inten = np.concatenate([il, ir]).astype(np.float32)
        ts = np.concatenate([tl, tr]).astype(np.float64)
        self.publish(xyz, inten, ts, c_left.header.stamp)

    def cb_imu_left_only(self, m):
        # Raw passthrough, no fusion -- kept symmetric with cb_left_only so
        # GLIM_LEFT_ONLY=1 is a fully self-consistent single-sensor baseline.
        m.header.frame_id = FRAME_ID
        self.imu_pub.publish(m)
        self.n_imu += 1

    def _buffer_right_imu(self, m):
        t_ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        self._right_imu_buf.append((
            t_ns,
            np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]),
            np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]),
            np.array(m.angular_velocity_covariance),
            np.array(m.linear_acceleration_covariance)))

    def _interp_right_imu(self, t_l):
        # Linearly extrapolate from the two most recent buffered right
        # samples to t_l. Left and right free-run at very slightly different
        # rates (measured ~5.047ms vs ~5.014ms period on this rig), so their
        # relative phase drifts through a full period every ~0.7s -- a fixed-
        # slop nearest-match (like the lidar path uses) drops ~30% of samples
        # in the bad part of that cycle no matter how it's tuned.
        #
        # This *extrapolates* rather than brackets-and-interpolates: true
        # bracketing needs a right sample *after* t_l, which in a live stream
        # hasn't been received yet when this callback fires (that's a
        # real-time/causality constraint the offline re-recorder doesn't
        # have -- it sees the whole bag at once and can bracket exactly).
        # t_l normally lands only ~half a period (~2.5ms) past the latest
        # buffered right sample, so linear extrapolation over that short a
        # span is a good approximation for a smooth IMU signal; frac is
        # typically just above 1.0, not a large extrapolation.
        #
        # Returns None if there's not enough buffered history yet (startup)
        # or the right stream has stalled for more than IMU_MAX_GAP_NS --
        # caller falls back to left-only rather than trust a stale/wild
        # extrapolation.
        buf = self._right_imu_buf
        if len(buf) < 2:
            return None
        t0, av0, la0, avcov0, lacov0 = buf[-2]
        t1, av1, la1, avcov1, lacov1 = buf[-1]
        if t1 <= t0:
            return None
        if (t_l - t1) > IMU_MAX_GAP_NS or (t0 - t_l) > IMU_MAX_GAP_NS:
            return None
        frac = (t_l - t0) / (t1 - t0)
        return (av0 + frac * (av1 - av0), la0 + frac * (la1 - la0),
                avcov0 + frac * (avcov1 - avcov0), lacov0 + frac * (lacov1 - lacov0))

    def cb_imu(self, m_left):
        # IMU and LiDAR are integrated in the same HAP enclosure, so the
        # right IMU is assumed to share its LiDAR's frame -- the lidar-only
        # rotation/lever-arm from EXTRINSICS applies directly to it too.
        R_lr = EXTRINSICS['right'][1]   # right -> left rotation
        r_lr = EXTRINSICS['right'][0]   # right IMU position in the left frame

        t_l = m_left.header.stamp.sec * 10**9 + m_left.header.stamp.nanosec
        av_l = np.array([m_left.angular_velocity.x, m_left.angular_velocity.y,
                          m_left.angular_velocity.z])
        la_l = np.array([m_left.linear_acceleration.x, m_left.linear_acceleration.y,
                          m_left.linear_acceleration.z])
        avcov_l = np.array(m_left.angular_velocity_covariance)
        lacov_l = np.array(m_left.linear_acceleration_covariance)

        interp = self._interp_right_imu(t_l)
        if interp is None:
            # No usable right-side bracket yet -- publish left alone rather
            # than drop the sample, so GLIM still sees a continuous ~199Hz
            # stream (this is a graceful "average of left with itself").
            av_r_raw, la_r_raw, avcov_r, lacov_r = av_l, la_l, avcov_l, lacov_l
        else:
            av_r_raw, la_r_raw, avcov_r, lacov_r = interp

        av_r = R_lr @ av_r_raw
        # Angular velocity is the same everywhere on a rigid body once
        # re-expressed in a common frame -- no lever-arm term, so this average
        # is exact (not an approximation), and denoises like any 2-sample mean.
        av = 0.5 * (av_l + av_r)

        la_r = R_lr @ la_r_raw
        # Specific force does differ across the ~2m lever arm between the two
        # IMUs: a_right = a_left + alpha x r + omega x (omega x r). We correct
        # the centripetal term (cheap, uses the gyro reading we already have)
        # and neglect the angular-acceleration term (would need differentiating
        # an already-noisy gyro signal -- not worth it for "simple" fusion).
        la_r_at_left = la_r - np.cross(av, np.cross(av, r_lr))
        la = 0.5 * (la_l + la_r_at_left)

        out = Imu()
        out.header.stamp = m_left.header.stamp
        out.header.frame_id = FRAME_ID
        out.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # no fused estimate
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = av.tolist()
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = la.tolist()
        # Two independent same-noise measurements averaged -> variance halves
        # (Var[(A+B)/2] = Var/2 for equal, independent A,B). Ignores the small
        # frame-rotation of the covariance tensor itself -- fine for a diagonal-
        # ish covariance, which is what the raw driver publishes.
        out.angular_velocity_covariance = (0.25 * (avcov_l + avcov_r)).tolist()
        out.linear_acceleration_covariance = (0.25 * (lacov_l + lacov_r)).tolist()
        self.imu_pub.publish(out)
        self.n_imu += 1
        if self.n_imu % 400 == 0:
            self.get_logger().info(f'fused {self.n_imu} IMU samples')

    def publish(self, xyz, inten, ts, stamp):
        xyz = np.asarray(xyz, dtype=np.float32)
        inten = np.asarray(inten, dtype=np.float32)
        ts = np.asarray(ts, dtype=np.float64)

        # Header stamp = earliest point time minus a 1 ms guard, so per-point
        # offsets (point_ts - header_stamp) are strictly positive downstream.
        # Reusing the LEFT header stamp made right-HAP points that lead the left
        # frame come out with negative offsets, which BIEVR's PointCloud2 path
        # casts through uint64 (wraps to ~1.8e19) and corrupts deskewing.
        if ts.size:
            t_min = float(ts.min())
            t_min_s = t_min * 1e-9 if t_min > 2.77e9 else t_min  # ns-vs-s, same heuristic as BIEVR
            t_min_s -= 1e-3
            sec = int(t_min_s)
            stamp = Time(sec=sec, nanosec=int((t_min_s - sec) * 1e9))
        out = np.zeros(xyz.shape[0], dtype=np.dtype({
            'names': ['x', 'y', 'z', 'intensity', 'timestamp'],
            'formats': ['<f4', '<f4', '<f4', '<f4', '<f8'],
            'offsets': [0, 4, 8, 12, 16], 'itemsize': 24}))
        out['x'], out['y'], out['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        out['intensity'] = inten
        out['timestamp'] = ts

        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=FRAME_ID)
        msg.height = 1
        msg.width = out.shape[0]
        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = 24 * out.shape[0]
        msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='timestamp', offset=16, datatype=PointField.FLOAT64, count=1),
        ]
        msg.data = out.tobytes()
        self.pub.publish(msg)
        self.n += 1
        if self.n % 20 == 0:
            self.get_logger().info(f'merged {self.n} scans ({out.shape[0]} pts last)')


def main():
    rclpy.init()
    rclpy.spin(Merger())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
