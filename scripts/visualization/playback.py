"""
This is a player for a set of synchronized RGB and depth images.

Run as:
python playback.py  --rgb_list rgb_aligned.txt --depth_list depth_aligned.txt --base_path /home/data/path/
or even
python playback.py  --base_path /home/data/path/
 
See also: assoc_rgbdi.py
"""

# Requirements:
#    pip install opencv-python
#    pip install numpy

import cv2
import numpy as np
import argparse
import os

def resolve_path(base_path, file_path):
    # Absolute path, leave unchanged
    if os.path.isabs(file_path):
        return file_path

    # Contains directory components, assume already relative path
    if os.path.dirname(file_path):
        return file_path

    # Plain filename, prepend base_path
    return os.path.join(base_path, file_path)

def read_list(file_path):
    entries = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            timestamp, filename = parts[0], parts[1]
            entries.append((timestamp, filename))
    return entries

def depth_to_8bit(depth):
    depth = depth.astype(np.float32)
    # ignore zeros
    mask = depth > 0

    if not np.any(mask):
        return np.zeros_like(depth, dtype=np.uint8)

    dmin = np.min(depth[mask])
    dmax = np.max(depth[mask])

    depth_norm = np.zeros_like(depth, dtype=np.float32)
    depth_norm[mask] = (depth[mask] - dmin) / (dmax - dmin + 1e-6)

    depth_8 = (depth_norm * 255).astype(np.uint8)
    return depth_8

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb_list", default="rgb_aligned.txt")
    parser.add_argument("--depth_list", default="depth_aligned.txt")
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--base_path", type=str, default="", required=False)  # optional prefix
    parser.add_argument("--start_frame", type=int, default=0)  # arbitrary first frame
    args = parser.parse_args()

    rgb_entries = read_list(resolve_path(args.base_path, args.rgb_list))
    depth_entries = read_list(resolve_path(args.base_path, args.depth_list))

    assert len(rgb_entries) == len(depth_entries), "RGB and depth lists must match"

    paused = False
    start_frame = max(0, min(args.start_frame, len(rgb_entries) - 1))
    i = start_frame
    while 0 <= i < len(rgb_entries):
        _, rgb_file = rgb_entries[i]
        _, depth_file = depth_entries[i]

        rgb_path = os.path.join(args.base_path, rgb_file)
        depth_path = os.path.join(args.base_path, depth_file)

        rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

        if rgb is None or depth is None:
            print(f"Skipping frame {i}, failed to load")
            continue

        # Convert depth to 8-bit
        depth_8 = depth_to_8bit(depth)
        #depth_8 = depth_to_8bit(depth, dmin=500, dmax=9000)
        depth_8 = cv2.createCLAHE(2.0, (8,8)).apply(depth_8)

        # Convert depth to 3-channel for display
        #depth_vis = cv2.cvtColor(depth_8, cv2.COLOR_GRAY2BGR)
        # Use pseudocolor for display
        depth_vis = cv2.applyColorMap(depth_8, cv2.COLORMAP_JET)

        # Resize if needed
        if args.scale != 1.0:
            rgb = cv2.resize(rgb, None, fx=args.scale, fy=args.scale)
            depth_vis = cv2.resize(depth_vis, None, fx=args.scale, fy=args.scale)

        # Concatenate imgs side by side
        combined = np.hstack((rgb, depth_vis))
        status = "PAUSED" if paused else rgb_file
        x, y = 20, 40
        font, thickness = cv2.FONT_HERSHEY_SIMPLEX, 2
        # black outline
        cv2.putText(combined, status, (x, y), font, 1, (0,0,0), thickness+2)
        cv2.putText(combined, status, (x, y), font, 1, (0,255,0), thickness)
        # depth filename
        #cv2.putText(combined, depth_file, (x+rgb.shape[1], y), font, 1, (0,0,255), thickness)

        cv2.imshow("RGB | Depth", combined)

        #key = cv2.waitKey(30)  # ~30 fps
        #if key == 27:  # ESC to quit
        #    break

        while True:
            key = cv2.waitKey(30 if not paused else 0) & 0xFF

            if key == 27:   # ESC to quit
                exit()
            elif key == ord(' '):  # SPACE toggle pause
                paused = not paused
            elif key == ord('n'):  # next frame
                i += 1
                break
            elif key == ord('b'):  # prev frame
                i = max(0, i - 1) 
                break
            elif key == ord('f'):  # jump forward 20
                i = min(len(rgb_entries) - 1, i + 20)
                break
            elif key == ord('r'):  # jump backward 20
                i = max(0, i - 20)
                break
            elif key != 255:  # key pressed but not recognized
                print("Help: [SPACE]=pause  n=next  b=back  f=+20  r=-20  ESC=quit")
                break

            if not paused:
                i += 1
                break  # move to next frame

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
