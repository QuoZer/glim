#!/usr/bin/env python3
"""Convert a GLIM map dump into HBA (hku-mars/HBA) input.

Each GLIM submap becomes one HBA scan:
  points_compact.bin (float32 xyz, submap ORIGIN frame)  ->  pcd/<i>.pcd
  T_world_origin (from data.txt)                          ->  a line in pose.json

Output layout (what HBA expects):
  <out>/pcd/0.pcd, 1.pcd, ...        (binary PCD, fields x y z intensity)
  <out>/pose.json                    (one line per pcd: tx ty tz qw qx qy qz)

Usage:
  python3 glim_to_hba.py [DUMP_DIR] [OUT_DIR]
    DUMP_DIR default: ~/glim_out
    OUT_DIR  default: ~/glim_hba
"""
import os
import sys
import glob
import struct
import numpy as np


def read_T_world_origin(data_txt):
    """Parse the 4x4 T_world_origin matrix from a submap data.txt."""
    with open(data_txt) as f:
        lines = f.readlines()
    for i, ln in enumerate(lines):
        if ln.startswith('T_world_origin:'):
            rows = [list(map(float, lines[i + 1 + r].split())) for r in range(4)]
            return np.array(rows, dtype=np.float64)
    raise ValueError(f'no T_world_origin in {data_txt}')


def rot_to_quat_wxyz(R):
    """3x3 rotation -> quaternion (w, x, y, z)."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    return w, x, y, z


def write_pcd_binary(path, xyz, intensity):
    """Write a binary PCD with fields x y z intensity (all float32)."""
    n = xyz.shape[0]
    data = np.zeros(n, dtype=np.dtype({
        'names': ['x', 'y', 'z', 'intensity'],
        'formats': ['<f4', '<f4', '<f4', '<f4'],
        'offsets': [0, 4, 8, 12], 'itemsize': 16}))
    data['x'], data['y'], data['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    data['intensity'] = intensity
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(data.tobytes())


def main():
    dump = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/glim_out')
    out = os.path.expanduser(sys.argv[2] if len(sys.argv) > 2 else '~/glim_hba')
    pcd_dir = os.path.join(out, 'pcd')
    os.makedirs(pcd_dir, exist_ok=True)

    submaps = sorted(d for d in glob.glob(os.path.join(dump, '[0-9]' * 6))
                     if os.path.isfile(os.path.join(d, 'data.txt')))
    if not submaps:
        sys.exit(f'no submaps (000000/data.txt ...) found in {dump}')

    pose_lines = []
    for i, sm in enumerate(submaps):
        T = read_T_world_origin(os.path.join(sm, 'data.txt'))
        pts = np.fromfile(os.path.join(sm, 'points_compact.bin'), dtype=np.float32).reshape(-1, 3)
        ip = os.path.join(sm, 'intensities_compact.bin')
        inten = (np.fromfile(ip, dtype=np.float32) if os.path.exists(ip)
                 else np.zeros(pts.shape[0], np.float32))
        if inten.shape[0] != pts.shape[0]:
            inten = np.zeros(pts.shape[0], np.float32)
        write_pcd_binary(os.path.join(pcd_dir, f'{i}.pcd'), pts, inten)
        t = T[:3, 3]
        w, x, y, z = rot_to_quat_wxyz(T[:3, :3])
        pose_lines.append(f'{t[0]:.9f} {t[1]:.9f} {t[2]:.9f} {w:.9f} {x:.9f} {y:.9f} {z:.9f}')
        print(f'  submap {i:3d}: {pts.shape[0]:7d} pts  t=[{t[0]:.2f} {t[1]:.2f} {t[2]:.2f}]')

    with open(os.path.join(out, 'pose.json'), 'w') as f:
        f.write('\n'.join(pose_lines) + '\n')

    print(f'\nWrote {len(submaps)} scans -> {pcd_dir}/*.pcd  and  {out}/pose.json')
    print('HBA data_path =', out)


if __name__ == '__main__':
    main()
