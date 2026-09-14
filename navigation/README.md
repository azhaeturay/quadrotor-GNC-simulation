# Navigation implementations

The final mission uses `simulate_imu.m`, `simulate_gps.m`, `simulate_magnetometer.m`, `navigation_ekf_init.m`, and `navigation_ekf_step.m`.

The EKF output interface returns `position`, `velocity`, `euler`, `accelNED`, `positionStd`, `velocityStd`, `attitudeStd`, `gpsUsed`, `gyroBias`, and `accelBias`. Bias fields are available on the initialization return as well as normal updates. Biases also remain available in `navState.x(10:12)` and `navState.x(13:15)`; the canonical mission retains its original direct-state logging. `gpsUsed` reports a normal GPS correction; initialization seeds from GPS but retains the existing false flag.

Earlier implementations remain in their original locations to preserve dependencies and make development comparisons accessible:

| Files | Development role |
|---|---|
| `kalman_1d_step.m` | Two-state position/velocity Kalman filter |
| `kalman_position_3d_step.m` | Six-state NED translation filter with acceleration input and GPS position updates |
| `attitude_estimator_step.m` | Gyro/accelerometer complementary attitude filter |
| `attitude_estimator_mag_step.m` | Complementary attitude filter with magnetometer heading aid |
| `navigation_estimator_init.m`, `navigation_estimator_step.m` | Earlier integrated complementary-attitude / translational-Kalman baseline |

These baselines are part of the development history, not alternative default estimators. See the [simulation index](../simulations/README.md) for their associated tests.
