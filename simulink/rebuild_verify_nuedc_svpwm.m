function report = rebuild_verify_nuedc_svpwm(attemptRapid)
%REBUILD_VERIFY_NUEDC_SVPWM Rebuild and structurally verify the SVPWM model.
%
% The pre-update backup must already exist. Generated caches are isolated
% under the system temporary directory, so stale Rapid Accelerator products
% from the project directory cannot satisfy the build.

if nargin < 1
    attemptRapid = true;
end
validateattributes(attemptRapid, {'logical'}, {'scalar'}, ...
    mfilename, 'attemptRapid');

scriptDir = fileparts(mfilename('fullpath'));
modelFile = fullfile(scriptDir, 'NUEDC2026A_SVPWM.slx');
backupFile = fullfile(scriptDir, ...
    'NUEDC2026A_SVPWM_before_safety_update.slx');
logDir = fullfile(scriptDir, 'validation_results');
assert(isfile(modelFile), 'Model not found: %s', modelFile);
assert(isfile(backupFile), 'Safety backup not found: %s', backupFile);
if ~isfolder(logDir)
    mkdir(logDir);
end

stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
logFile = fullfile(logDir, ['rebuild_verify_' stamp '.log']);
reportFile = fullfile(logDir, ['rebuild_verify_' stamp '.mat']);
assert(~isfile(logFile) && ~isfile(reportFile), ...
    'Refusing to overwrite an existing verification record.');

diary(logFile);
diaryCleanup = onCleanup(@() diary('off'));
fprintf('NUEDC SVPWM rebuild verification\n');
fprintf('Started: %s\n', char(datetime('now')));
fprintf('MATLAB: %s\n', version);
fprintf('Model: %s\n', modelFile);
fprintf('Backup: %s\n', backupFile);

oldFileGen = Simulink.fileGenControl('getConfig');
cacheRoot = fullfile(tempdir, ['NUEDC_SVPWM_' stamp]);
cacheFolder = fullfile(cacheRoot, 'cache');
codegenFolder = fullfile(cacheRoot, 'codegen');
runFolder = fullfile(cacheRoot, 'run');
mkdir(cacheFolder);
mkdir(codegenFolder);
mkdir(runFolder);
Simulink.fileGenControl('set', ...
    'CacheFolder', cacheFolder, ...
    'CodeGenFolder', codegenFolder, ...
    'CodeGenFolderStructure', oldFileGen.CodeGenFolderStructure, ...
    'createDir', true);
fileGenCleanup = onCleanup(@() restore_filegen(oldFileGen));
oldPath = path;
pathCleanup = onCleanup(@() path(oldPath));
addpath(scriptDir);
oldFolder = pwd;
folderCleanup = onCleanup(@() cd(oldFolder));
cd(runFolder);
fprintf('Isolated cache root: %s\n', cacheRoot);
fprintf('Isolated run folder: %s\n', runFolder);

report = struct( ...
    'started', char(datetime('now')), ...
    'modelFile', modelFile, ...
    'backupFile', backupFile, ...
    'logFile', logFile, ...
    'cacheRoot', cacheRoot, ...
    'modelRebuilt', false, ...
    'updatePassed', false, ...
    'compilePassed', false, ...
    'rapidAcceleratorAttempted', attemptRapid, ...
    'rapidAcceleratorBuilt', false, ...
    'completed', '');

try
    modify_nuedc_svpwm;
    report.modelRebuilt = true;
    fprintf('MODEL_REBUILD: PASS\n');

    [~, mdl] = fileparts(modelFile);
    load_system(modelFile);
    modelCleanup = onCleanup(@() close_if_loaded(mdl));

    set_param(mdl, 'SimulationCommand', 'update');
    report.updatePassed = true;
    fprintf('MODEL_UPDATE: PASS\n');

    compiled = false;
    try
        feval(mdl, [], [], [], 'compile');
        compiled = true;
        feval(mdl, [], [], [], 'term');
        compiled = false;
        report.compilePassed = true;
        fprintf('MODEL_COMPILE: PASS\n');
    catch cause
        if compiled
            try
                feval(mdl, [], [], [], 'term');
            catch
            end
        end
        rethrow(cause);
    end

    if attemptRapid
        Simulink.BlockDiagram.buildRapidAcceleratorTarget(mdl);
        report.rapidAcceleratorBuilt = true;
        fprintf('RAPID_ACCELERATOR_BUILD: PASS\n');
    else
        fprintf('RAPID_ACCELERATOR_BUILD: SKIPPED BY CALLER\n');
    end

    report.completed = char(datetime('now'));
    save(reportFile, 'report');
    fprintf('Report MAT: %s\n', reportFile);
    fprintf('Completed: %s\n', report.completed);
catch cause
    fprintf(2, 'REBUILD_OR_VERIFY: FAIL\n%s\n', ...
        getReport(cause, 'extended', 'hyperlinks', 'off'));
    report.completed = char(datetime('now'));
    save(reportFile, 'report', 'cause');
    rethrow(cause);
end
end

function restore_filegen(config)
Simulink.fileGenControl('set', ...
    'CacheFolder', config.CacheFolder, ...
    'CodeGenFolder', config.CodeGenFolder, ...
    'CodeGenFolderStructure', config.CodeGenFolderStructure, ...
    'createDir', true);
end

function close_if_loaded(mdl)
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
end
