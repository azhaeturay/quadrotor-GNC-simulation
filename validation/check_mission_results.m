function check_mission_results(r)
%CHECK_MISSION_RESULTS Deterministic regression, not a robustness campaign.
% Call after main: check_mission_results(results)
root = fileparts(fileparts(mfilename('fullpath')));
reference = jsondecode(fileread(fullfile(root, 'validation', 'reference_metrics.json')));
assert(r.missionComplete, 'The mission did not complete.');
assert(r.segment(end) == size(r.waypoints,1)-1, 'Final segment not reached.');
assert(norm(r.positionEstimate(:,end)-r.waypoints(end,:).') < ...
    r.parameters.waypointAcceptanceRadius, 'Estimated final position criterion failed.');
assert(norm(r.velocityEstimate(:,end)) < r.parameters.waypointAcceptanceSpeed, ...
    'Estimated final speed criterion failed.');
arrays = {r.stateTrue, r.positionEstimate, r.velocityEstimate, ...
    r.attitudeEstimate, r.rotorThrust, r.gyroBiasEstimate, r.accelBiasEstimate};
for k = 1:numel(arrays)
    assert(all(isfinite(arrays{k}), 'all'), 'Nonfinite mission history.');
    assert(size(arrays{k},2) == numel(r.time), 'History length mismatch.');
end
assert(all(r.rotorThrust >= r.parameters.minRotorThrust-1e-12, 'all'));
assert(all(r.rotorThrust <= r.parameters.maxRotorThrust+1e-12, 'all'));
assert(norm(r.navigationFinalCovariance-r.navigationFinalCovariance', 'fro') < 1e-10);
assert(min(eig(r.navigationFinalCovariance)) >= -1e-10);
assert(abs(norm(r.positionTrue(:,end)-r.waypoints(end,:).') - ...
    r.metrics.trueFinalWaypointError_m) < 1e-12, 'Final distance definition mismatch.');
% Small tolerances accommodate platform roundoff; this checks the fixed seed only.
fields = {'duration_s', 'trueCrossTrackRMS_m', 'trueCrossTrackMax_m', ...
    'trueFinalWaypointError_m', 'estimatedCrossTrackRMS_m', ...
    'estimatedCrossTrackMax_m', 'estimatedFinalWaypointError_m', ...
    'positionRMSE_m', 'velocityRMSE_mps', 'attitudeRMSE_deg'};
tolerances = [0.02, 0.002, 0.005, 0.005, 0.002, 0.005, 0.005, 0.002, 0.002, 0.02];
for k = 1:numel(fields)
    actual = r.metrics.(fields{k});
    expected = reference.metrics.(fields{k});
    assert(all(abs(actual(:)-expected(:)) <= tolerances(k)), ...
        'Reference comparison failed for %s.', fields{k});
end
fprintf('PASS: mission completion, finite histories, actuator bounds, final covariance, and reference metrics.\n');
end
