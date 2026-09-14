# Simulation and validation index

Start with [`test_estimated_state_mission.m`](test_estimated_state_mission.m), also launched by root-level [`main.m`](../main.m). It closes the full GNC loop using the 15-state EKF. The other scripts preserve the incremental development sequence; their settings and mathematics have been retained.

From the repository root in MATLAB:

```matlab
addpath(genpath(pwd))
run('simulations/test_dynamics.m')  % Replace with any script below
```

The scripts clear their workspace and close existing figures. Most are interactive engineering checks with printed comparisons and plots, not assertion-based unit tests. Their scenario settings differ; do not substitute their numbers for the final mission's reference results.

| Stage | Scripts | Purpose |
|---|---|---|
| Plant | [`test_dynamics.m`](test_dynamics.m) | Free fall, exact hover, initial roll acceleration |
| Rotor model | [`test_rotor_plant.m`](test_rotor_plant.m), [`test_allocation.m`](test_allocation.m) | Rotor-level hover/roll response, wrench mapping, inverse allocation |
| Inner loops | [`test_rate_controller.m`](test_rate_controller.m), [`test_attitude_controller.m`](test_attitude_controller.m), [`test_attitude_3axis.m`](test_attitude_3axis.m) | Rate response against a first-order prediction; attitude recovery and three-axis tracking |
| Outer loops | [`test_altitude_controller.m`](test_altitude_controller.m), [`test_position_controller.m`](test_position_controller.m) | Altitude and XYZ setpoint tracking with truth feedback |
| Guidance | [`test_waypoint_mission.m`](test_waypoint_mission.m), [`test_lookahead_mission.m`](test_lookahead_mission.m) | Stop-at-waypoint and fly-through path-following development with truth feedback |
| Sensors | [`test_sensors.m`](test_sensors.m), [`test_sensor_timing.m`](test_sensor_timing.m), [`test_sensor_stream.m`](test_sensor_stream.m) | Noise/bias observations, free-fall specific force, update scheduling, synthetic sensor stream |
| Linear filtering | [`test_kalman_1d.m`](test_kalman_1d.m), [`test_kalman_3d.m`](test_kalman_3d.m) | Position/velocity prediction, GPS correction, error/covariance diagnostics; the 3-D isolation test uses known attitude |
| Attitude estimation | [`test_attitude_estimator.m`](test_attitude_estimator.m), [`test_attitude_estimator_mag.m`](test_attitude_estimator_mag.m) | Gyro-only versus complementary filtering, then magnetometer-aided heading |
| Earlier navigation baseline | [`test_navigation_estimator.m`](test_navigation_estimator.m) | Complementary attitude plus translational Kalman filtering; this is not the final coupled EKF |
| Estimated-state control | [`test_estimated_state_hover.m`](test_estimated_state_hover.m) | 15-state EKF feedback hover from an offset position and attitude |
| Final integrated mission | [`test_estimated_state_mission.m`](test_estimated_state_mission.m) | Estimated-state takeoff/square mission, offline metrics, saved data, and figure export |

For a one-pass execution check of all 19 earlier scripts:

```matlab
addpath('validation')
report = run_subsystem_validations;
```

This checks execution and dependency resolution. It is not a Monte Carlo run or a blanket assertion that every historical scenario meets a performance specification. The runner writes `results/subsystem_execution.json`; the [validation record](../validation/README.md) distinguishes these checks from the final mission regression.
