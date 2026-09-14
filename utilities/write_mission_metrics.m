function write_mission_metrics(results, filename)
%WRITE_MISSION_METRICS Export measured metrics and execution provenance.
record.metadata = results.metadata;
record.missionComplete = results.missionComplete;
record.metrics = results.metrics;
record.finalTruePositionNED_m = results.positionTrue(:,end);
record.finalWaypointNED_m = results.waypoints(end,:).';
record.finalEstimatedSpeed_mps = norm(results.velocityEstimate(:,end));
fid = fopen(filename, 'w');
assert(fid >= 0, 'Unable to open metrics output: %s', filename);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(record, 'PrettyPrint', true));
end
