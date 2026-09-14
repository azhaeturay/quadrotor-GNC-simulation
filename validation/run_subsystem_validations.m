function report = run_subsystem_validations()
%RUN_SUBSYSTEM_VALIDATIONS Execute the 19 preserved development scripts once.
% This is an execution/path smoke check, not automatic performance approval.
% Each script prints/plots its original diagnostics in a separate workspace.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
scripts = dir(fullfile(root, 'simulations', 'test_*.m'));
scripts(strcmp({scripts.name}, 'test_estimated_state_mission.m')) = [];
report = struct('script', {}, 'completedWithoutError', {}, 'error', {});
for k = 1:numel(scripts)
    report(k).script = scripts(k).name;
    try
        run_one(fullfile(scripts(k).folder, scripts(k).name));
        report(k).completedWithoutError = true;
        report(k).error = '';
    catch err
        report(k).completedWithoutError = false;
        report(k).error = err.message;
    end
    close all;
end
outputDir = fullfile(root, 'results');
if ~isfolder(outputDir), mkdir(outputDir); end
fid = fopen(fullfile(outputDir, 'subsystem_execution.json'), 'w');
assert(fid >= 0, 'Unable to open subsystem report.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(report, 'PrettyPrint', true));
fprintf('\nSUBSYSTEM EXECUTION SUMMARY\n');
for k = 1:numel(report)
    fprintf('%d  %s  %s\n', report(k).completedWithoutError, report(k).script, report(k).error);
end
assert(all([report.completedWithoutError]), 'A preserved script failed; inspect results/subsystem_execution.json.');
fprintf('All %d preserved scripts completed without execution errors. Inspect their diagnostics separately.\n', numel(report));
end

function run_one(scriptPath)
% Isolate the scripts' clear statements from the runner's loop and report.
run(scriptPath);
end
