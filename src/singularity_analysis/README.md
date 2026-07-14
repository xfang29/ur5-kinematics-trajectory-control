# Singularity Analysis Extension

This folder adds an offline comparison between the existing pen-tip
Moore–Penrose pseudoinverse controller and a damped least-squares (DLS)
inverse.

## Placement

Copy this folder to:

```text
ur5-kinematics-trajectory-control/
└─ src/
   ├─ DH.m
   ├─ urFwdKin.m
   ├─ calculatePenJacobian.m
   └─ singularity_analysis/
      ├─ dlsJointVelocity.m
      ├─ so3LogVector.m
      ├─ simulateRRNearSingularity.m
      ├─ run_singularity_sweep.m
      └─ run_rr_near_singularity_comparison.m
```

## Run order

Run these scripts from MATLAB:

```matlab
run_singularity_sweep
run_rr_near_singularity_comparison
```

Both scripts automatically add the parent `src` directory to the MATLAB
path.

## Experiment 1: instantaneous singularity sweep

`run_singularity_sweep.m` reduces `q3` from `0.50` rad toward `0.01` rad
while keeping the other joints fixed. A pure base-frame `+Y` pen-tip
velocity is mapped to joint velocity using:

```matlab
dq_pinv = pinv(J_pen) * V_cmd;
dq_dls  = J_pen' * ((J_pen*J_pen' + lambda^2*eye(6)) \ V_cmd);
```

The script records:

- minimum singular value;
- condition number;
- maximum joint-speed demand;
- joint-speed norm;
- task-space velocity residual.

The sweep is intentionally not integrated. It isolates the inverse
mapping itself from trajectory and saturation effects.

## Experiment 2: closed-loop near-singular trajectory

`run_rr_near_singularity_comparison.m` simulates a straight pen-tip
motion with fixed orientation. The pseudoinverse and DLS controllers use
identical Cartesian feedback and the same `0.23 rad/s` whole-vector
proportional speed scaling used by the original controller.

The script records:

- maximum, RMS, and final pen-tip position error;
- raw and applied joint-speed peaks;
- peak finite-difference joint acceleration;
- joint-velocity total variation;
- minimum singular value;
- speed-saturation count.

## Important interpretation

DLS solves a regularized least-squares problem. It is expected to reduce
joint-speed amplification near a singularity, but it generally introduces
a nonzero task-space velocity residual. The comparison should therefore
report the tracking-versus-robustness tradeoff rather than claiming that
DLS is unconditionally more accurate.

The singular values of a 6D geometric Jacobian combine translational and
rotational rows with different physical units. They remain useful for
consistent within-model comparisons and rank-loss detection, but the
absolute numerical threshold is tied to the chosen units and scaling.
