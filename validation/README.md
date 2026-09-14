# Validation record

**Date:** 14 September 2026

**Environment:** MATLAB R2026a Update 5, macOS Apple silicon (`MACA64`)

**Reference:** canonical estimated-state mission, `rng(12)` with the `twister` generator

## Scope and evidence

The local source folder was the authoritative implementation. All 50 original MATLAB files and the parameter autosave were inspected before structural changes. The reusable modules and all 20 original simulation scripts retain their names and locations. The unmodified source was recorded in a separate import commit before packaging edits.

| Check | Outcome |
|---|---|
| Unmodified canonical mission in local MATLAB | Completed at 22.58 s |
| Cleaned mission through root `main.m` | Completed at 22.58 s |
| Truth state, navigation estimates, commands, bias and cross-track histories | Exactly equal to the saved pre-cleanup run, sample by sample |
| Final EKF state and covariance | Exactly equal to the saved pre-cleanup run |
| Final-segment and estimated position/speed acceptance | Passed |
| Finite logged histories, rotor bounds, final covariance symmetry/PSD | Passed |
| Initialization / prediction / GPS navigation output interface | Passed |
| MATLAB dependency report for `main.m` | MATLAB only; no archived code resolved |
| 19 earlier development scripts, one execution each | Completed without execution errors |
| Seven exported 300-dpi PNGs | Generated in MATLAB and visually inspected |

Evidence: [unmodified mission console](baseline_console.txt), [successful cleanup regression console](cleanup_console.txt), [reference metrics](reference_metrics.json), and [subsystem execution report](subsystem_execution.json).

The history comparison covered time, the 12-state plant trajectory, estimated position/velocity/attitude, rotor commands, attitude commands, both bias histories, both cross-track histories, and active segment indices. Final EKF state/covariance equality was checked separately. The preserved complete workspace snapshots remain local, outside the published source set.

The script execution report is a dependency and execution smoke check. Many original scripts rely on printed analytical comparisons and visual inspection; successful execution alone is not a performance acceptance test. No new Monte Carlo, wind, or GPS-dropout campaign was performed.

## Metric definitions and reference correction

The final true waypoint error is the full three-dimensional Euclidean distance in NED:

```matlab
norm(positionTrueHistory(:,end) - waypoints(end,:).')
```

The measured final true position is approximately:

```text
[-0.025007556205; 0.163387020811; -2.030887532070] m
```

The final waypoint is `[0; 0; -2]` m, giving **0.168150932421 m**, rounded to **0.168 m**. The unmodified rerun and the preexisting MATLAB workspace both give this value. Prior project notes supplied 0.108 m as the final true waypoint error; that value is not supported by this calculation. The README uses the measured value. No simulation changes were made to reproduce the prior number.

The separate **true RMS cross-track error remains 0.108001379645 m**. Cross-track distance is measured to the active finite 3-D segment, rather than the final waypoint or the closest segment anywhere in the path. Switching segments can introduce jumps in this metric. The onboard value is computed at guidance updates and held between them. All logged samples, including initialization, contribute to the reported RMS values. Attitude errors are wrapped to [−π, π) before RMSE; displayed yaw is unwrapped for readable tracking plots.

There are **7 clipped command samples out of 2,259 logged samples**. This count is reconstructed offline by applying the existing allocation matrix to the recorded desired wrench and checking whether any unconstrained rotor command exceeds its 0–8 N limits. It is a sample count, not a claim of seven separate saturation events. The mission completes despite brief clipping around sharp transitions.

## Changes under regression

- `main.m` now launches the canonical mission instead of displaying parameters.
- The canonical mission explicitly adds executable source folders, keeping archives/output out of its dependency path.
- Plotting moved to `utilities/plot_mission_results.m`; simulation, logging, and original metric calculations remain unchanged. Yaw unwrapping affects display only.
- Structured histories and metrics are exported after the simulation, including offline clipping counts.
- `navigation_ekf_step.m` now assigns `gyroBias` and `accelBias` inside the common output helper. This makes the initialization and normal interfaces consistent without changing state or covariance mathematics. The mission retains direct bias logging from `navState.x`.

The cleanup regression initially detected that a loose local source backup could shadow live functions under recursive `genpath`. The backup was preserved as an archive and non-executable `.m.original` files, and the canonical mission's paths were made explicit. The successful checks above were rerun with the intended live EKF and verified dependency resolution.

All controller gains, estimator tuning, sensor noise, vehicle parameters, guidance logic, RK4 integration, random seed, and waypoints remain unchanged. No meaningful validation scripts were deleted or moved. `config/vehicle_params.asv` remains locally preserved but ignored; macOS metadata, MATLAB temporary files, local backups, and generated run data are excluded by `.gitignore`.

## Reproduce

From the repository root in MATLAB:

```matlab
main
check_mission_results(results)
test_navigation_output
report = run_subsystem_validations;
```

`check_mission_results` compares the fixed-seed run with the committed full-precision reference metrics and checks completion, finite histories, actuator bounds, and final covariance. It allows small numerical differences: 0.02 s duration, 0.002 m RMS/position-RMSE, 0.005 m peak/final distances, 0.002 m/s velocity-RMSE, and 0.02° attitude-RMSE. These are regression tolerances, not design requirements or robustness guarantees. Exact history equality was an additional one-time comparison to the original local workspace.

The 19-script runner closes figures after each script and writes `results/subsystem_execution.json`. Run scripts individually to inspect their plots. `main` regenerates the final mission figures afterward. The existing `rng(12)` call is preserved; the recorded run used `twister`. If your MATLAB default generator differs, select `rng(12,'twister')` before running `main` for this reference comparison.

This is software-in-the-loop simulation validation, not flight testing or hardware qualification.
