#!/usr/bin/env python3
"""Compare GLIM vs HBA-optimized poses and (optionally) export merged clouds.

HBA overwrites pose.json with poses RELATIVE TO SCAN 0. We compose them back to
world using the original scan-0 world pose (kept in pose_input.json) so the
before/after are directly comparable.

Usage:
  python3 hba_result.py [DATA_DIR] [--merge]
    DATA_DIR default: ~/glim_hba
    --merge : also write map_glim.pcd and map_hba.pcd (voxel-downsampled world
              clouds) for side-by-side viewing.
"""
import os
import sys
import numpy as np


def quat_wxyz_to_R(w, x, y, z):
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z),   2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def load_poses(path):
    """Return list of 4x4 world/relative transforms from a pose.json."""
    Ts = []
    for ln in open(path):
        v = ln.split()
        if len(v) < 7:
            continue
        tx, ty, tz, qw, qx, qy, qz = map(float, v[:7])
        T = np.eye(4)
        T[:3, :3] = quat_wxyz_to_R(qw, qx, qy, qz)
        T[:3, 3] = [tx, ty, tz]
        Ts.append(T)
    return Ts


def read_pcd_bin(path):
    """Read the binary PCD written by glim_to_hba.py (x y z intensity f32)."""
    with open(path, 'rb') as f:
        while True:
            line = f.readline().decode('ascii', 'replace')
            if line.startswith('POINTS'):
                n = int(line.split()[1])
            if line.startswith('DATA'):
                break
        buf = f.read()
    arr = np.frombuffer(buf, dtype=np.float32, count=n*4).reshape(-1, 4)
    return arr[:, :3].astype(np.float64)


def voxel_ds(pts, leaf=0.3):
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def write_pcd(path, pts):
    n = pts.shape[0]
    hdr = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
           f"COUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           f"POINTS {n}\nDATA binary\n")
    with open(path, 'wb') as f:
        f.write(hdr.encode('ascii'))
        f.write(pts.astype(np.float32).tobytes())


def main():
    data = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else '~/glim_hba')
    do_merge = '--merge' in sys.argv

    init = load_poses(os.path.join(data, 'pose_input.json'))   # world (GLIM)
    hba_rel = load_poses(os.path.join(data, 'pose.json'))       # relative to scan 0
    T_w0 = init[0]                                              # scan-0 world pose (fixed by HBA)
    hba_world = [T_w0 @ T for T in hba_rel]

    print(f'{"i":>3} {"z_glim":>10} {"z_hba":>10} {"dz":>8}')
    zi, zh = [], []
    for i, (A, B) in enumerate(zip(init, hba_world)):
        zi.append(A[2, 3]); zh.append(B[2, 3])
        print(f'{i:3d} {A[2,3]:10.3f} {B[2,3]:10.3f} {B[2,3]-A[2,3]:8.3f}')
    zi, zh = np.array(zi), np.array(zh)
    print(f'\nZ span  GLIM: {zi.max()-zi.min():8.2f} m   HBA: {zh.max()-zh.min():8.2f} m')
    print(f'Z drift (last-first)  GLIM: {zi[-1]-zi[0]:8.2f} m   HBA: {zh[-1]-zh[0]:8.2f} m')

    if do_merge:
        for name, poses in [('map_glim.pcd', init), ('map_hba.pcd', hba_world)]:
            allpts = []
            for i, T in enumerate(poses):
                p = read_pcd_bin(os.path.join(data, 'pcd', f'{i}.pcd'))
                allpts.append(p @ T[:3, :3].T + T[:3, 3])
            merged = voxel_ds(np.vstack(allpts), 0.3)
            write_pcd(os.path.join(data, name), merged)
            print(f'wrote {os.path.join(data, name)}  ({merged.shape[0]} pts)')


if __name__ == '__main__':
    main()
