"""One-off lens calibration for the Sensor Unit's visible camera.

    python tools/calibrate_lens.py photos/*.jpg  --cols 9 --rows 6 --square 25

Print a checkerboard (cols x rows INNER corners, square size in mm), tape it
to something flat, and take 15-25 photos with the Sensor Unit camera at its
working resolution (2304 x 1296), tilting the board and covering the corners
of the frame. Writes lens.npz (K, D) next to run_pi.py; run_pi.py then
undistorts every pixel before the ground mapping.

Accept the result if the RMS reprojection error is under about 0.5 px.
"""
import argparse
import glob
import os

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="+")
    ap.add_argument("--cols", type=int, default=9, help="inner corners per row")
    ap.add_argument("--rows", type=int, default=6, help="inner corners per column")
    ap.add_argument("--square", type=float, default=25.0, help="square size, mm")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "lens.npz"))
    a = ap.parse_args()

    objp = np.zeros((a.rows * a.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    obj_pts, img_pts, size = [], [], None
    files = [f for pat in a.images for f in glob.glob(pat)]
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        size = img.shape[::-1]
        ok, corners = cv2.findChessboardCorners(img, (a.cols, a.rows))
        if not ok:
            print("no board:", f)
            continue
        corners = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3))
        obj_pts.append(objp)
        img_pts.append(corners)
    if len(img_pts) < 10:
        raise SystemExit(f"only {len(img_pts)} usable photos; take at least 10 (15-25 is better)")
    rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None,
                                          flags=cv2.CALIB_RATIONAL_MODEL)
    np.savez(a.out, K=K, D=D)
    print(f"{len(img_pts)} photos, RMS reprojection error {rms:.2f} px -> {os.path.abspath(a.out)}")
    if rms > 0.5:
        print("RMS is high: retake photos with the board sharp, flat and filling the frame corners")


if __name__ == "__main__":
    main()
