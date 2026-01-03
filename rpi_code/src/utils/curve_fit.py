import cv2
import numpy as np

def _contour_to_points(contour):
    pts = contour.reshape(-1, 2)
    # ensure closed
    if not np.array_equal(pts[0], pts[-1]):
        pts = np.vstack([pts, pts[0]])
    return pts.astype(np.float32)

def _simplify_points(points, epsilon=2.0):
    cnt = points.reshape(-1, 1, 2).astype(np.int32)
    approx = cv2.approxPolyDP(cnt, epsilon, True).reshape(-1, 2).astype(np.float32)
    if len(approx) < 3:  # fallback
        approx = points
    if not np.array_equal(approx[0], approx[-1]):
        approx = np.vstack([approx, approx[0]])
    return approx

def _cumulative_lengths(points):
    diffs = np.diff(points, axis=0)
    seg_lens = np.linalg.norm(diffs, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
    return cum, seg_lens

def _uniform_resample_closed(points, num_points):
    cum, seg_lens = _cumulative_lengths(points)
    total = cum[-1]
    if total < 1e-6 or len(points) < 2:
        return points.astype(np.float32)
    targets = np.linspace(0, total, num_points, endpoint=False)
    resampled = []
    j = 0
    for t in targets:
        while j < len(cum) - 2 and cum[j + 1] < t:
            j += 1
        t0, t1 = cum[j], cum[j + 1]
        p0, p1 = points[j], points[j + 1]
        if t1 - t0 < 1e-6:
            resampled.append(p0)
        else:
            u = (t - t0) / (t1 - t0)
            resampled.append((1 - u) * p0 + u * p1)
    return np.array(resampled, dtype=np.float32)

def _chaikin_closed(points, iterations=2):
    pts = points.astype(np.float32)
    for _ in range(max(0, iterations)):
        new_pts = []
        n = len(pts)
        for i in range(n - 1):
            p = pts[i]
            q = pts[(i + 1) % n]
            new_pts.append(0.75 * p + 0.25 * q)
            new_pts.append(0.25 * p + 0.75 * q)
        new_pts = np.array(new_pts, dtype=np.float32)
        # ensure closed
        if not np.array_equal(new_pts[0], new_pts[-1]):
            new_pts = np.vstack([new_pts, new_pts[0]])
        pts = new_pts
    return pts

def fit_smooth_closed_curve(contour, num_points=150, simplify_epsilon=2.0, chaikin_iters=2):
    pts = _contour_to_points(contour)
    pts = _simplify_points(pts, epsilon=simplify_epsilon)
    pts = _uniform_resample_closed(pts, max(20, num_points // 2))
    pts = _chaikin_closed(pts, iterations=chaikin_iters)
    pts = _uniform_resample_closed(pts, num_points)
    return pts.astype(np.int32)

def draw_curve(image, points, color=(0, 255, 0), thickness=2):
    pts = points.reshape(-1, 1, 2).astype(np.int32)
    cv2.polylines(image, [pts], isClosed=True, color=color, thickness=thickness)
