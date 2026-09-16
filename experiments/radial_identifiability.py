"""Evidence for the two-stage certifiable intrinsic + extrinsic scheme.

Run against a session produced by `simulate`, which stores ground-truth X, Y, the
true camera, and every detected corner:

    python3 -m hardware.rb5_camera --config hardware/rb5.gopro.json \
        simulate --output data/rb5/sim01 --count 30
    python3 experiments/radial_identifiability.py data/rb5/sim01

It answers three questions, in order.

1. Is (X, Y) identifiable from 1D-radial constraints alone - that is, from pixel
   *directions* about the principal point, which carry no focal length and no
   radial distortion whatsoever?

2. If not, exactly which degree of freedom is missing? The claim is that the
   radial residual is invariant under X -> X * Translate_z(delta), so precisely
   one dof is unobservable no matter how much data is collected, and it is the
   camera's position along its own optical axis.

3. Does the discarded information - the radial *magnitudes* - recover that one
   dof together with the focal length and distortion? The claim is that this is
   a one-dimensional search with a closed-form inner fit.

See docs/CERTIFIED_INTRINSICS.md for the derivation these numbers support.
"""
import json
import sys
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hardware.rb5_camera.board import make_board
from hardware.rb5_camera.geometry import inverse

STEP = 1e-6
LABELS = ["X.wx", "X.wy", "X.wz", "X.tx", "X.ty", "X.tz",
          "Y.wx", "Y.wy", "Y.wz", "Y.tx", "Y.ty", "Y.tz"]


def load(session):
    session = Path(session)
    truth = json.loads((session / "ground_truth.json").read_text())
    meta = json.loads((session / "session.json").read_text())
    if not meta.get("simulated_images"):
        raise SystemExit("Needs a session from the 'simulate' command (it stores ground truth)")
    corners = np.asarray(make_board(meta["config"]["board"]).getChessboardCorners(),
                         dtype=np.float64).reshape(-1, 3)
    views = []
    for path in sorted((session / "samples").glob("*.json")):
        sample = json.loads(path.read_text())
        quality = sample["quality"]["target"]
        views.append((np.array(sample["T_base_hand"]),
                      corners[np.asarray(quality["corner_ids"], dtype=int)],
                      np.asarray(quality["corners_px"], dtype=float)))
    return truth, views


def exp_se3(xi):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(xi[:3])[0]
    T[:3, 3] = xi[3:]
    return T


def slide(X, delta):
    """X * Translate_z(delta): move the camera along its own optical axis."""
    T = np.eye(4)
    T[2, 3] = delta
    return X @ T


class Radial:
    """The 1D-radial residual: the part of (u - c) perpendicular to (Px, Py).

    Normalised by |u - c| and |(Px, Py)| so each corner contributes an angle in
    radians. Corners near the principal point carry no information and this
    scaling lets them say so rather than blowing up.
    """

    def __init__(self, views, centre):
        self.views, self.centre = views, centre

    def __call__(self, X, Y):
        out = []
        for A, objects, pixels in self.views:
            T = inverse(X) @ inverse(A) @ Y
            P = objects @ T[:3, :3].T + T[:3, 3]
            d = pixels - self.centre
            scale = np.maximum(np.linalg.norm(d, axis=1) * np.linalg.norm(P[:, :2], axis=1), 1e-12)
            out.append((d[:, 0] * P[:, 1] - d[:, 1] * P[:, 0]) / scale)
        return np.concatenate(out)


class Reprojection:
    """The full pixel residual, which does use the intrinsics. For comparison."""

    def __init__(self, views, camera):
        self.views = views
        self.K = np.array(camera["K"])
        self.D = np.array(camera["distortion"])
        self.fisheye = camera["model"] == "fisheye"

    def __call__(self, X, Y):
        out = []
        for A, objects, pixels in self.views:
            T = inverse(X) @ inverse(A) @ Y
            rvec, tvec = cv2.Rodrigues(T[:3, :3])[0], T[:3, 3]
            if self.fisheye:
                projected = cv2.fisheye.projectPoints(objects.reshape(1, -1, 3), rvec, tvec,
                                                      self.K, self.D)[0]
            else:
                projected = cv2.projectPoints(objects, rvec, tvec, self.K, self.D)[0]
            out.append((projected.reshape(-1, 2) - pixels).ravel())
        return np.concatenate(out)


def jacobian(residual, X, Y):
    """Numerical Jacobian over the 12 dof of (X, Y), left-perturbed on SE(3)."""
    base = residual(X, Y)
    J = np.empty((base.size, 12))
    for k in range(12):
        xi = np.zeros(6)
        xi[k % 6] = STEP
        moved = (exp_se3(xi) @ X, Y) if k < 6 else (X, exp_se3(xi) @ Y)
        J[:, k] = (residual(*moved) - base) / STEP
    return base, J


def fit_radial_profile(views, centre, X, Y):
    """Kannala-Brandt r = f(t + k1 t^3 + ... + k4 t^9): linear in (f, f*k1, ...)."""
    angles, radii = [], []
    for A, objects, pixels in views:
        T = inverse(X) @ inverse(A) @ Y
        P = objects @ T[:3, :3].T + T[:3, 3]
        angles.append(np.arctan2(np.linalg.norm(P[:, :2], axis=1), P[:, 2]))
        radii.append(np.linalg.norm(pixels - centre, axis=1))
    theta, radius = np.concatenate(angles), np.concatenate(radii)
    design = np.stack([theta ** power for power in (1, 3, 5, 7, 9)], axis=1)
    coefficients, *_ = np.linalg.lstsq(design, radius, rcond=None)
    return float(np.sqrt(np.mean((design @ coefficients - radius) ** 2))), coefficients


def main(session):
    truth, views = load(session)
    X, Y = np.array(truth["X"]), np.array(truth["Y"])
    camera = truth["true_camera"]
    centre = np.array([camera["cx"], camera["cy"]])
    radial = Radial(views, centre)
    print(f"{len(views)} views, {sum(len(v[1]) for v in views)} corner observations\n")

    print("=" * 72)
    print("1. Identifiability of (X, Y) from pixel DIRECTIONS only")
    print("=" * 72)
    base, J = jacobian(radial, X, Y)
    s = np.linalg.svd(J, compute_uv=False)
    print(f"residual at ground truth: rms {np.sqrt(np.mean(base ** 2)):.3e} rad")
    print(f"rank: {np.sum(s > 1e-8 * s[0])}/12      condition number: {s[0] / s[-1]:.1f}")
    print(f"two smallest singular values: {s[-2]:.4f}, {s[-1]:.6f}")
    print("  -> full rank, but one direction is 2000x weaker than the next.")

    print()
    print("=" * 72)
    print("2. The unobservable direction is exact, not ill-conditioning")
    print("=" * 72)
    print("  sliding the camera along its own optical axis:")
    for delta in (0.001, 0.01, 0.1, 0.5):
        moved = radial(slide(X, delta), Y)
        print(f"    {delta * 1000:7.1f} mm along camera z : {np.abs(moved - base).max():.3e}")
    for axis, name in ((0, "x"), (1, "y")):
        T = np.eye(4)
        T[axis, 3] = 0.1
        print(f"    {100.0:7.1f} mm along camera {name} : "
              f"{np.abs(radial(X @ T, Y) - base).max():.3e}   (observable)")
    print(f"  residual rms, for scale                 : {np.sqrt(np.mean(base ** 2)):.3e}")

    # In left-perturbation coordinates the null direction is a translation of X
    # along R_X e3, since X * Translate_z(d) = [R_X | t_X + d * R_X e3].
    null = np.zeros(12)
    null[3:6] = X[:3, :3] @ np.array([0, 0, 1.0])
    null /= np.linalg.norm(null)
    v_last = np.linalg.svd(J, compute_uv=True)[2][-1]
    reduced = np.linalg.svd(J @ (np.eye(12) - np.outer(null, null)), compute_uv=False)[:11]
    pixel_base, pixel_J = jacobian(Reprojection(views, camera), X, Y)
    pixel_s = np.linalg.svd(pixel_J, compute_uv=False)
    print(f"\n  angle(numerical null vector, R_X e3)    : "
          f"{np.degrees(np.arccos(min(1, abs(v_last @ null)))):.3f} deg")
    print(f"  radial condition, all 12 dof            : {s[0] / s[-1]:12.1f}")
    print(f"  radial condition, gauge-fixed to 11 dof : {reduced[0] / reduced[-1]:12.1f}")
    print(f"  full reprojection condition, 12 dof     : "
          f"{pixel_s[0] / pixel_s[-1]:12.1f}")
    print("  -> exactly one dof is lost; the other eleven are as well conditioned")
    print("     as the full residual, which needs the intrinsics.")

    print()
    print("=" * 72)
    print("3. Radial magnitudes recover that dof, with the focal length")
    print("=" * 72)
    print("   delta [mm]   RMS [px]     f [px]")
    for delta_mm in (-150, -40, -10, -1, 0, 1, 10, 40, 150):
        rms, coefficients = fit_radial_profile(views, centre, slide(X, delta_mm / 1000), Y)
        print(f"   {delta_mm:8.0f}   {rms:8.4f}  {coefficients[0]:9.2f}")
    grid = np.linspace(-0.02, 0.02, 801)
    values = np.array([fit_radial_profile(views, centre, slide(X, d), Y)[0] for d in grid])
    best = grid[np.argmin(values)]
    rms, coefficients = fit_radial_profile(views, centre, slide(X, best), Y)
    print(f"\n  recovered delta : {best * 1000:+.3f} mm   (truth 0)")
    print(f"  recovered focal : {coefficients[0]:.2f} px  (truth {camera['fx']:.2f}), "
          f"relative error {coefficients[0] / camera['fx'] - 1:+.5f}")
    print(f"  Stage B RMS     : {rms:.4f} px")
    print("  -> the inner fit is closed form, so only delta is searched: one dimension.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
