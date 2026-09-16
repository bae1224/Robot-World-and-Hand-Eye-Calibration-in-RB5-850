"""Per-sample precisions for the maximum likelihood objective.

Problem 1 of the paper carries an individual Langevin concentration kappa_i and
translation variance sigma_i^2 for every measurement, but the RB5 entry point has
been passing one constant for all of them (kappa=125, sigma=0.01 m), which the
documentation admits are guesses rather than anything measured. That throws away
real information: a board seen small, far away or nearly edge-on yields a pose
that is genuinely an order of magnitude less certain than one seen large and
tilted, and the MLE should know.

These weights come from the Gauss-Newton covariance of each view's PnP solve
(board.refine_pose_pixels), so they reflect corner count, board geometry, viewing
distance and tilt at no extra cost.

Two caveats, stated rather than hidden:

* The paper's model assumes A (forward kinematics) is noiseless and puts all
  error into B. Joint-angle and model error therefore leak into a residual that
  these weights attribute entirely to the camera, so the absolute scale of kappa
  and tau is optimistic. The *relative* weighting between samples, which is what
  actually changes the estimate, is unaffected.
* The covariance is first-order and assumes independent, identically distributed
  corner noise. Correlated detection error (motion blur, rolling shutter, JPEG
  blocking from a capture box) is not modelled.
"""
import numpy as np

# src/calibration/robot_world_costs.jl accumulates kappa_i * M'M for rotation and
# tau_i * M'M for translation, i.e. without the 1/2 that Equations (18a)-(18b)
# carry. Matching the negative log-likelihood therefore gives:
#   rotation:     kappa * ||R_A R_X - R_Y R_B||_F^2 ~ kappa * 2 ||delta||^2
#                 equals ||delta||^2 / (2 sigma_R^2)  ->  kappa = 1 / (4 sigma_R^2)
#   translation:  tau * ||.||^2 equals ||.||^2 / (2 sigma_t^2)  ->  tau = 1 / (2 sigma_t^2)
MIN_ROTATION_SIGMA_RAD = 1e-5
MIN_TRANSLATION_SIGMA_M = 1e-6
# Clamp so one implausibly confident view cannot dominate every other sample.
MAX_WEIGHT_RATIO = 1e4


def sample_weights(sample, T_camera_target):
    """(kappa, tau) for one exported sample, or None when no covariance was stored."""
    quality = sample.get("quality")
    if not isinstance(quality, dict):
        return None
    target = quality.get("target", quality)
    sigma_rotation = target.get("rotation_sigma_deg")
    sigma_translation = target.get("translation_sigma_m")
    if sigma_rotation is None or sigma_translation is None:
        return None
    sigma_rotation = max(np.deg2rad(float(sigma_rotation)), MIN_ROTATION_SIGMA_RAD)
    sigma_translation = max(float(sigma_translation), MIN_TRANSLATION_SIGMA_M)
    if not np.isfinite([sigma_rotation, sigma_translation]).all():
        return None
    # The covariance describes T_camera_target, but B is its inverse for an
    # eye-in-hand rig. Inversion leaves the rotation uncertainty alone and adds a
    # lever-arm term to the translation: t_B = -R' t, so an angular error of
    # sigma_R displaces the origin by about ||t|| sigma_R.
    lever = float(np.linalg.norm(np.asarray(T_camera_target, dtype=float)[:3, 3]))
    sigma_translation = float(np.hypot(sigma_translation, lever * sigma_rotation))
    return [1.0 / (4.0 * sigma_rotation ** 2), 1.0 / (2.0 * sigma_translation ** 2)]


def clamp_weights(rows):
    """Limit the spread so a single optimistic view cannot swamp the objective."""
    values = np.asarray(rows, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or not np.isfinite(values).all():
        raise ValueError("weights must be an N x 2 array of finite kappa, tau")
    for column in range(2):
        ceiling = values[:, column].min() * MAX_WEIGHT_RATIO
        values[:, column] = np.minimum(values[:, column], ceiling)
    return values
