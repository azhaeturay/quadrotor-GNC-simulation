# Reference figures

These PNGs were exported directly by MATLAB at 300 dpi from the canonical `rng(12)` mission. The [reference metrics](../validation/reference_metrics.json) record the run environment and measured results.

Run root-level `main.m` to regenerate them. To plot saved results without rerunning the simulation:

```matlab
addpath('utilities')
load('results/full_gnc_mission.mat', 'results')
plot_mission_results(results)
```

The plot utility leaves the sampled data unchanged. Yaw is unwrapped only for display. The rotor-thrust plot retains the brief clipping at actuator limits.
