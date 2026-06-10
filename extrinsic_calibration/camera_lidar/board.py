"""ChArUco board detection -> board pose + plane in the camera frame.

Uses the OpenCV >= 4.7 `cv2.aruco.CharucoDetector` API (we run 4.13). A detection
returns the full board pose `T_cam_board` (board origin = outer corner of the
first square, x along columns, y along rows, z out of the pattern face) plus the
board plane as `(n, d)` with `n . X + d = 0` for points X on the board in camera
coordinates and `n` oriented TOWARD the camera (n . centroid < 0, d > 0).

Plane convention is shared with plane_solve.py — keep them in sync.
"""
import cv2
import numpy as np


class BoardDetector:
    def __init__(self, squares_x, squares_y, square_len_m, marker_len_m,
                 dictionary="DICT_5X5_100", min_corners=8):
        if not hasattr(cv2.aruco, dictionary):
            raise ValueError(f"Unknown ArUco dictionary '{dictionary}'")
        self.dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.board = cv2.aruco.CharucoBoard(
            (int(squares_x), int(squares_y)), float(square_len_m),
            float(marker_len_m), self.dict)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.CharucoDetector(
            self.board, detectorParams=params)
        self.min_corners = int(min_corners)
        self.size_m = (squares_x * square_len_m, squares_y * square_len_m)

    def detect(self, image, K, D, fisheye=False):
        """Detect the board in one image.

        image: BGR or grayscale ndarray. K: 3x3 intrinsics. D: distortion coeffs
        (plumb_bob for pinhole, k1..k4 for fisheye=True).

        Returns None if the board is not (reliably) seen, else a dict with:
          T_cam_board : 4x4 board pose in the camera frame
          n, d        : board plane, n . X + d = 0, n toward camera
          corners_obj : (N,3) used ChArUco corners in the board frame
          corners_img : (N,2) their image positions
          reproj_err  : RMS reprojection error in pixels
        """
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _, _ = self.detector.detectBoard(gray)
        if ch_ids is None or len(ch_ids) < self.min_corners:
            return None
        obj_pts, img_pts = self.board.matchImagePoints(ch_corners, ch_ids)
        if obj_pts is None or len(obj_pts) < self.min_corners:
            return None
        obj_pts = obj_pts.reshape(-1, 3).astype(np.float64)
        img_pts = img_pts.reshape(-1, 2).astype(np.float64)

        K = np.asarray(K, float).reshape(3, 3)
        D = np.asarray(D, float).ravel()
        if fisheye:
            # Undistort corner pixels to normalized image coords, then solve a
            # distortion-free PnP (identity intrinsics).
            und = cv2.fisheye.undistortPoints(
                img_pts.reshape(-1, 1, 2), K, D[:4].reshape(-1, 1))
            pnp_img, pnp_K, pnp_D = und.reshape(-1, 2), np.eye(3), None
        else:
            pnp_img, pnp_K, pnp_D = img_pts, K, D

        # Planar target -> IPPE is the right PnP flavour; it needs >= 4 points.
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, pnp_img, pnp_K, pnp_D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, pnp_img, pnp_K, pnp_D, rvec=rvec, tvec=tvec,
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None

        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, pnp_K,
                                    pnp_D if pnp_D is not None else np.zeros(5))
        err = float(np.sqrt(np.mean(np.sum(
            (proj.reshape(-1, 2) - pnp_img) ** 2, axis=1))))

        Rm, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = tvec.ravel()

        # Plane through the board: normal = board z-axis in camera coords. Use the
        # detected-corner centroid (not the board origin) as the plane point and
        # orient n toward the camera.
        centroid = (Rm @ obj_pts.mean(axis=0)) + tvec.ravel()
        n = Rm[:, 2].copy()
        if float(n @ centroid) > 0:
            n = -n
        d = -float(n @ centroid)

        return {
            "T_cam_board": T,
            "n": n, "d": d,
            "corners_obj": obj_pts,
            "corners_img": img_pts,
            "reproj_err": err,
            "n_corners": len(obj_pts),
        }
