% 对尚未覆盖主模型的候选build文件执行36 V/60 Hz/2 A标称验收。
scriptDir = fileparts(mfilename('fullpath'));
logDir = fullfile(scriptDir, 'validation_results');
if ~isfolder(logDir)
    mkdir(logDir);
end
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
logFile = fullfile(logDir, ['candidate_nominal_' stamp '.log']);
diary(logFile);
diaryCleanup = onCleanup(@() diary('off')); %#ok<NASGU>

oldFolder = pwd;
folderCleanup = onCleanup(@() cd(oldFolder)); %#ok<NASGU>
runFolder = tempname;
mkdir(runFolder);
cd(runFolder);
addpath(scriptDir);

[results, artifacts] = run_nuedc_svpwm_acceptance( ...
    [], [], ...
    'NominalOnly', true, ...
    'ModelFile', fullfile(scriptDir, 'NUEDC2026A_SVPWM_build.slx'), ...
    'OutputDirectory', fullfile(logDir, 'candidate_nominal'));
disp(results);
disp(artifacts);
