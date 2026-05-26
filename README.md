# openarm_control

Reusable kinematics and control utilities for OpenArm, backed by MuJoCo and [mink](https://github.com/kevinzakka/mink).

## Install

```bash
uv sync
```

## Usage

### `Kinematics`


```python
from openarm_control import Kinematics, IKParams, ArmSetup

# FK only
kin = Kinematics(setup)
pose = kin.fk("right", joints)               # float32[7]
pose_r, pose_l = kin.fk_bimanual(r, l)       # single mj_forward

# IK
kin = Kinematics(setup, IKParams(damping=0.25, posture_cost=0.01))
kin.set_target("right", pose_r)
kin.set_target("left", pose_l)
result = kin.solve(dt=0.1, n_iters=5)        # float32[16] right[8]+left[8]
```

### `IKParams`

Solver configuration passed to `Kinematics`. All fields have defaults.

| Field | Default | Description |
|---|---|---|
| `position_cost` | `1.0` | Position task weight |
| `orientation_cost` | `1.0` | Orientation task weight |
| `lm_damping` | `0.01` | Per-task Levenberg-Marquardt damping |
| `damping` | `0.25` | Global Tikhonov regularization |
| `solver` | `"daqp"` | QP backend |
| `posture_cost` | `0.01` | Neutral posture task weight (0 = disabled) |
| `diag_reg` | `0.0` | QP diagonal regularization |
| `dt` | `0.1` | Integration timestep per iteration |
| `max_iters` | `5` | IK iterations per solve |
| `velocity_limits` | `None` | Per-joint velocity caps in mink units (`None` = disabled) |
| `avoid_collisions` | `False` | Enable the mink `CollisionAvoidanceLimit` (CBF) in the QP |
| `collision_margin` | `0.005` | Minimum clearance kept between geoms, meters |
| `collision_sensor` | `0.02` | Detection band at which the barrier engages, meters |

Build from CLI args with `register_ik_args` + `ik_params_from_args`:

Pass `--vel-scale S` to enable velocity limits: each arm joint is capped at `ARM_JOINT_VELOCITY_LIMITS_RAD_S × S` (rad/s preset in `config.py`), converted to mink units via `--tick-hz`. Omit `--vel-scale` to disable.

Pass `--avoid-collisions` to add a `CollisionAvoidanceLimit` (control barrier function) to the IK QP, keeping the arms clear of each other, the body, and the environment. `--collision-margin` sets the minimum clearance maintained between geoms; `--collision-sensor` sets the detection band at which the barrier starts acting and must exceed the margin. Collision geom pairs are built automatically from the model by `geom_pairs_for_arms`. Omit `--avoid-collisions` to disable.
