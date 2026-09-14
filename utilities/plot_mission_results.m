function figures = plot_mission_results(r, outputDir)
%PLOT_MISSION_RESULTS Plot saved mission data and export publication PNGs.
% No filtering, smoothing, resampling, or feedback changes are applied.
% Yaw is unwrapped for display only; error metrics retain wrapped differences.
% Usage: load('results/full_gnc_mission.mat'); plot_mission_results(results);
if nargin < 2
    root = fileparts(fileparts(mfilename('fullpath')));
    outputDir = fullfile(root, 'figures');
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
blue = [0.05 0.32 0.56];
orange = [0.84 0.36 0.10];
green = [0.12 0.52 0.39];
gray = [0.40 0.44 0.49];
t = r.time;
p = r.positionTrue;
ph = r.positionEstimate;
w = r.waypoints;
figures = gobjects(7,1);

figures(1) = new_figure('Mission top view', [850 750]);
plot(w(:,2), w(:,1), 'o--', 'Color', gray, 'LineWidth', 1.2, ...
    'MarkerFaceColor', 'w', 'MarkerSize', 6); hold on;
plot(p(2,:), p(1,:), 'Color', blue, 'LineWidth', 2);
plot(ph(2,:), ph(1,:), '--', 'Color', orange, 'LineWidth', 1.1);
plot(p(2,1), p(1,1), 's', 'Color', green, 'MarkerFaceColor', green, 'MarkerSize', 8);
plot(p(2,end), p(1,end), 'd', 'Color', blue, 'MarkerFaceColor', blue, 'MarkerSize', 7);
axis equal; grid on;
allNE = [w(:,1:2); p(1:2,:).'; ph(1:2,:).'];
low = min(allNE); high = max(allNE); margin = 0.06*(high-low);
xlim([low(2)-margin(2), high(2)+margin(2)]);
ylim([low(1)-margin(1), high(1)+margin(1)]);
xlabel('East [m]'); ylabel('North [m]');
title('Estimated-state square mission');
subtitle(sprintf('%.2f s  |  True cross-track RMS %.3f m  |  Seed %d', ...
    r.metrics.duration_s, r.metrics.trueCrossTrackRMS_m, r.metadata.seed));
legend('Waypoint path', 'True vehicle', 'Navigation estimate', ...
    'Start', 'Finish', 'Location', 'southoutside', 'NumColumns', 3);
export_figure(figures(1), outputDir, 'full_gnc_mission_top_view.png');

figures(2) = new_figure('Mission 3-D trajectory', [950 720]);
plot3(w(:,2), w(:,1), -w(:,3), 'o--', 'Color', gray, 'LineWidth', 1.2, ...
    'MarkerFaceColor', 'w', 'MarkerSize', 6); hold on;
plot3(p(2,:), p(1,:), -p(3,:), 'Color', blue, 'LineWidth', 2);
plot3(ph(2,:), ph(1,:), -ph(3,:), '--', 'Color', orange, 'LineWidth', 1.1);
axis equal; grid on; view(40,27);
xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude = -Down [m]');
title('Takeoff and 4 m square at 2 m altitude');
subtitle('Nonlinear 6-DOF plant with estimated-state feedback');
legend('Waypoint path', 'True vehicle', 'Navigation estimate', ...
    'Location', 'southoutside', 'Orientation', 'horizontal');
export_figure(figures(2), outputDir, 'full_gnc_mission_3d.png');

figures(3) = new_figure('Navigation estimation errors', [1000 850]);
layout = tiledlayout(3,1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'Navigation estimation error', 'Color', [0.15 0.18 0.22]);
subtitle(layout, 'Estimate minus truth; full run including initialization', 'Color', [0.15 0.18 0.22]);
errors = {r.positionEstimate-r.positionTrue, ...
    r.velocityEstimate-r.velocityTrue, ...
    rad2deg(mod(r.attitudeEstimate-r.attitudeTrue+pi,2*pi)-pi)};
labels = {'Position error [m]', 'Velocity error [m/s]', 'Attitude error [deg]'};
for j = 1:3
    nexttile; colororder([blue; orange; green]);
    plot(t, errors{j}', 'LineWidth', 1.0); grid on;
    ylabel(labels{j}); xlim([t(1) t(end)]);
    if j < 3
        legend('North', 'East', 'Down', 'Location', 'northeast');
    else
        legend('Roll', 'Pitch', 'Yaw', 'Location', 'northeast');
        xlabel('Time [s]');
    end
end
export_figure(figures(3), outputDir, 'navigation_estimation_error.png');

figures(4) = new_figure('Attitude tracking', [1000 850]);
layout = tiledlayout(3,1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'Cascaded attitude control', 'Color', [0.15 0.18 0.22]);
subtitle(layout, 'True attitude, navigation estimate, and commanded attitude', 'Color', [0.15 0.18 0.22]);
truth = rad2deg(r.attitudeTrue);
estimate = rad2deg(r.attitudeEstimate);
command = rad2deg(r.attitudeCommand);
truth(3,:) = rad2deg(unwrap(r.attitudeTrue(3,:)));
estimate(3,:) = rad2deg(unwrap(r.attitudeEstimate(3,:)));
command(3,:) = rad2deg(unwrap(r.attitudeCommand(3,:)));
labels = {'Roll [deg]', 'Pitch [deg]', 'Yaw, unwrapped [deg]'};
for j = 1:3
    nexttile;
    plot(t, truth(j,:), 'Color', blue, 'LineWidth', 1.6); hold on;
    plot(t, estimate(j,:), '--', 'Color', orange, 'LineWidth', 1.0);
    plot(t, command(j,:), ':', 'Color', gray, 'LineWidth', 1.3);
    grid on; ylabel(labels{j}); xlim([t(1) t(end)]);
    if j == 1
        legend('Truth', 'Estimate', 'Command', 'Location', 'northeast', 'NumColumns', 3);
    elseif j == 3
        xlabel('Time [s]');
    end
end
export_figure(figures(4), outputDir, 'attitude_tracking.png');

figures(5) = new_figure('Cross-track error', [1000 480]);
plot(t, r.crossTrackTrue, 'Color', blue, 'LineWidth', 1.6); hold on;
plot(t, r.crossTrackEstimate, '--', 'Color', orange, 'LineWidth', 1.1);
for j = 1:numel(r.transitionTimes)
    xline(r.transitionTimes(j), ':', 'Color', gray, 'HandleVisibility', 'off');
end
grid on; xlim([t(1) t(end)]);
xlabel('Time [s]'); ylabel('Distance to active segment [m]');
title('Path-following error');
subtitle('3-D finite-segment distance; dotted vertical lines mark segment changes');
legend('Truth at 100 Hz', 'Onboard estimate held from 50 Hz guidance', ...
    'Location', 'northeast');
export_figure(figures(5), outputDir, 'cross_track_error.png');

figures(6) = new_figure('Rotor thrust and allocation limits', [1000 500]);
colororder([blue; orange; green; 0.52 0.30 0.65]);
plot(t, r.rotorThrust', 'LineWidth', 1.0); hold on;
yline(r.parameters.minRotorThrust, '--', 'Color', gray, 'HandleVisibility', 'off');
yline(r.parameters.maxRotorThrust, '--', 'Color', gray, 'HandleVisibility', 'off');
grid on; xlim([t(1) t(end)]); ylim([-0.4 r.parameters.maxRotorThrust+0.4]);
xlabel('Time [s]'); ylabel('Rotor thrust [N]');
title('Control allocation and actuator clipping');
subtitle(sprintf('%d of %d command samples clipped; dashed lines show actuator limits', ...
    r.metrics.clippedCommandSamples, r.metrics.commandSamples));
legend('Rotor 1', 'Rotor 2', 'Rotor 3', 'Rotor 4', ...
    'Location', 'southoutside', 'Orientation', 'horizontal');
export_figure(figures(6), outputDir, 'rotor_thrusts.png');

% Retain the original mission-speed diagnostic as a secondary output.
figures(7) = new_figure('Mission speed', [1000 480]);
plot(t, vecnorm(r.velocityTrue), 'Color', blue, 'LineWidth', 1.6); hold on;
plot(t, vecnorm(r.velocityEstimate), '--', 'Color', orange, 'LineWidth', 1.1);
grid on; xlim([t(1) t(end)]);
xlabel('Time [s]'); ylabel('Speed [m/s]'); title('Mission speed');
legend('Truth', 'Estimate', 'Location', 'northeast');
export_figure(figures(7), outputDir, 'mission_speed.png');
end

function f = new_figure(name, sizePx)
f = figure('Name', name, 'NumberTitle', 'off', 'Color', 'w', ...
    'Units', 'pixels', 'Position', [80 80 sizePx], ...
    'DefaultAxesFontName', 'Helvetica', 'DefaultAxesFontSize', 11, ...
    'DefaultAxesColor', 'w', 'DefaultAxesXColor', [0.15 0.18 0.22], ...
    'DefaultAxesYColor', [0.15 0.18 0.22], ...
    'DefaultAxesZColor', [0.15 0.18 0.22], ...
    'DefaultTextColor', [0.15 0.18 0.22], ...
    'DefaultAxesGridColor', [0.35 0.40 0.45], ...
    'DefaultAxesGridAlpha', 0.14, 'DefaultAxesBox', 'on');
end

function export_figure(f, outputDir, name)
set(findall(f, 'Type', 'legend'), 'Color', 'w', 'TextColor', [0.15 0.18 0.22], 'Box', 'off');
drawnow;
exportgraphics(f, fullfile(outputDir, name), 'Resolution', 300, 'BackgroundColor', 'white');
end
