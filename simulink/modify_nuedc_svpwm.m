function modify_nuedc_svpwm()
% Create a corrected SVPWM simulation from NUEDC2026A_23b.slx.
% The source model is never overwritten.

scriptDir = fileparts(mfilename('fullpath'));
srcFile = fullfile(scriptDir, 'NUEDC2026A_23b.slx');
dstFile = fullfile(scriptDir, 'NUEDC2026A_SVPWM.slx');
srcMdl = 'NUEDC2026A_23b';
mdl = 'NUEDC2026A_SVPWM';

assert(isfile(srcFile), 'Source model not found: %s', srcFile);
assert(~isfile(dstFile), ...
    'Destination already exists. Move it aside before rebuilding: %s', dstFile);

if bdIsLoaded(srcMdl)
    close_system(srcMdl, 0);
end
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

load_system(srcFile);
save_system(srcMdl, dstFile);
close_system(srcMdl, 0);
load_system(dstFile);

cleanup = onCleanup(@() close_if_loaded(mdl));

%% Model parameters
mws = get_param(mdl, 'ModelWorkspace');
assignin(mws, 'Ui_rms', 36);
assignin(mws, 'fi', 50);
assignin(mws, 'Ull_rms', 32);
assignin(mws, 'Io_rms', 2);
assignin(mws, 'f_out', 60);
assignin(mws, 'Vdc_ref', 60);
assignin(mws, 'fsw', 20000);
assignin(mws, 'Ts_power', 1e-6);
assignin(mws, 'L_filter', 1.1e-3);
assignin(mws, 'C_filter', 10e-6);
assignin(mws, 'P_unmodelled_W', 0);

% The power network is already discrete at 1 us. A fixed-step solver avoids
% excessive zero-crossing refinement at every switching edge and makes the
% simulation deterministic.
set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode3', ...
    'FixedStep', 'Ts_power');

set_sid(mdl, 15, 'Amplitude', 'Ui_rms*sqrt(2)', 'Frequency', 'fi');
set_sid(mdl, 31, 'Value', 'Vdc_ref');
set_sid(mdl, 136, 'Value', 'f_out');
set_sid(mdl, 147, 'Value', 'Ull_rms/sqrt(3)');

% Input and output series inductors: two at the input and three at the output.
for sid = [5 6 7 8 9]
    set_sid(mdl, sid, 'Inductance', 'L_filter');
end

% Input shunt capacitor and three delta-connected output capacitors.
for sid = [17 24 25 26]
    set_sid(mdl, sid, 'Capacitance', 'C_filter');
end

% Keep the original 4.4 mF DC-link capacitor (SID 4) until its hardware value
% is provided.

% Correct the star-connected scoring load.
for sid = [182 184 185]
    set_sid(mdl, sid, ...
        'Resistance', 'Ull_rms/(sqrt(3)*Io_rms)');
end

% Correct resonant-controller frequencies.
set_sid(mdl, 43, 'Wr', '2*pi*fi');
for sid = [153 203 187 211]
    set_sid(mdl, sid, 'Wr', '2*pi*f_out');
end

% Power-network and carrier settings.
set_sid(mdl, 69, ...
    'SampleTime', 'Ts_power', ...
    'frequency', 'fi', ...
    'fundamental', 'fi');
set_sid(mdl, 65, ...
    'rep_seq_t', '[0 1/fsw/2 1/fsw]');
set_sid(mdl, 323, ...
    'rep_seq_t', '[0 1/fsw/2 1/fsw]');

% Measurement block frequencies.
for sid = [38 349 350 352 353]
    set_sid(mdl, sid, 'Freq', 'fi');
end
for sid = [378 252 253 379 380 258 259 247]
    set_sid(mdl, sid, 'Freq', 'f_out');
end

%% Replace DPWM-A zero-sequence generation with continuous SVPWM
zeroSequence = getfullname(sid_handle(mdl, 285));
Simulink.SubSystem.deleteContents(zeroSequence);

add_block('simulink/Ports & Subsystems/In1', ...
    [zeroSequence '/abc'], ...
    'Port', '1', ...
    'Position', [30 93 60 107]);
add_block('simulink/Signal Routing/Demux', ...
    [zeroSequence '/Demux'], ...
    'Outputs', '3', ...
    'Position', [100 45 105 155]);
add_block('simulink/Math Operations/MinMax', ...
    [zeroSequence '/Max'], ...
    'Function', 'max', ...
    'Inputs', '3', ...
    'Position', [170 35 205 75]);
add_block('simulink/Math Operations/MinMax', ...
    [zeroSequence '/Min'], ...
    'Function', 'min', ...
    'Inputs', '3', ...
    'Position', [170 125 205 165]);
add_block('simulink/Math Operations/Sum', ...
    [zeroSequence '/MaxPlusMin'], ...
    'Inputs', '++', ...
    'Position', [250 82 275 118]);
add_block('simulink/Math Operations/Gain', ...
    [zeroSequence '/MinusHalf'], ...
    'Gain', '-0.5', ...
    'Position', [315 83 365 117]);
add_block('simulink/Ports & Subsystems/Out1', ...
    [zeroSequence '/zero_sequence'], ...
    'Port', '1', ...
    'Position', [410 93 440 107]);

add_line(zeroSequence, 'abc/1', 'Demux/1', 'autorouting', 'on');
for k = 1:3
    add_line(zeroSequence, sprintf('Demux/%d', k), ...
        sprintf('Max/%d', k), 'autorouting', 'on');
    add_line(zeroSequence, sprintf('Demux/%d', k), ...
        sprintf('Min/%d', k), 'autorouting', 'on');
end
add_line(zeroSequence, 'Max/1', 'MaxPlusMin/1', 'autorouting', 'on');
add_line(zeroSequence, 'Min/1', 'MaxPlusMin/2', 'autorouting', 'on');
add_line(zeroSequence, 'MaxPlusMin/1', 'MinusHalf/1', 'autorouting', 'on');
add_line(zeroSequence, 'MinusHalf/1', 'zero_sequence/1', 'autorouting', 'on');

% deleteContents removes the original Inport/Outport blocks and leaves
% their parent-level line stubs behind. Explicitly reconnect the recreated
% zero-sequence subsystem at the parent level.
modulator = getfullname(sid_handle(mdl, 283));
vabcPorts = get_param(sid_handle(mdl, 284), 'PortHandles');
zeroSequencePorts = get_param(sid_handle(mdl, 285), 'PortHandles');
sum2Ports = get_param(sid_handle(mdl, 322), 'PortHandles');
disconnect_port_line(vabcPorts.Outport(1));
disconnect_destination(sum2Ports.Inport(2));
add_line(modulator, vabcPorts.Outport(1), ...
    sum2Ports.Inport(3), 'autorouting', 'on');
add_line(modulator, vabcPorts.Outport(1), ...
    zeroSequencePorts.Inport(1), 'autorouting', 'on');
add_line(modulator, zeroSequencePorts.Outport(1), ...
    sum2Ports.Inport(2), 'autorouting', 'on');

set_param(sid_handle(mdl, 285), 'Name', 'SVPWM Zero Sequence');
set_param(sid_handle(mdl, 283), 'Name', 'SVPWM Modulator');

%% Correct current measurement locations
% Insert an ideal current sensor ahead of the input shunt capacitor. The
% existing IL sensor remains after the capacitor and continues to feed the
% PFC current loop.
inputTotalSensor = add_block(getfullname(sid_handle(mdl, 10)), ...
    [mdl '/Iin Total Sensor'], ...
    'Position', [365 398 390 422]);

sourcePorts = get_param(sid_handle(mdl, 15), 'PortHandles');
inputVoltagePorts = get_param(sid_handle(mdl, 14), 'PortHandles');
inputInductorSensorPorts = get_param(sid_handle(mdl, 10), 'PortHandles');
inputCapPorts = get_param(sid_handle(mdl, 17), 'PortHandles');
inputTotalSensorPorts = get_param(inputTotalSensor, 'PortHandles');

% Rebuild the upper input node as:
% source -> total-current sensor -> {input capacitor, IL sensor}.
disconnect_port_line(sourcePorts.RConn(1));
disconnect_port_line(inputVoltagePorts.LConn(1));
disconnect_port_line(inputInductorSensorPorts.LConn(1));
disconnect_port_line(inputCapPorts.LConn(1));
add_line(mdl, sourcePorts.RConn(1), inputTotalSensorPorts.LConn(1), ...
    'autorouting', 'on');
add_line(mdl, sourcePorts.RConn(1), inputVoltagePorts.LConn(1), ...
    'autorouting', 'on');
add_line(mdl, inputTotalSensorPorts.RConn(1), ...
    inputInductorSensorPorts.LConn(1), 'autorouting', 'on');
add_line(mdl, inputTotalSensorPorts.RConn(1), inputCapPorts.LConn(1), ...
    'autorouting', 'on');

inputMetrics = get_param(sid_handle(mdl, 340), 'PortHandles');
disconnect_destination(inputMetrics.Inport(2));
add_line(mdl, inputTotalSensorPorts.Outport(1), ...
    inputMetrics.Inport(2), 'autorouting', 'on');

%% Correct total input power factor: PF = P / (Urms * Irms)
inputMetricsPath = getfullname(sid_handle(mdl, 340));
voltageCurrentProduct = add_block('simulink/Math Operations/Product', ...
    [inputMetricsPath '/UrmsIrms'], ...
    'Inputs', '**', ...
    'Position', [310 390 345 425]);
pfDenominatorGuard = add_block('simulink/Discontinuities/Saturation', ...
    [inputMetricsPath '/PF Denominator Guard'], ...
    'LowerLimit', 'eps', ...
    'UpperLimit', 'inf', ...
    'Position', [365 390 405 425]);

rmsVoltage = get_param(sid_handle(mdl, 349), 'PortHandles');
rmsCurrent = get_param(sid_handle(mdl, 350), 'PortHandles');
pfDivide = get_param(sid_handle(mdl, 357), 'PortHandles');
productPorts = get_param(voltageCurrentProduct, 'PortHandles');
pfGuardPorts = get_param(pfDenominatorGuard, 'PortHandles');
disconnect_destination(pfDivide.Inport(2));
add_line(inputMetricsPath, rmsVoltage.Outport(1), ...
    productPorts.Inport(1), 'autorouting', 'on');
add_line(inputMetricsPath, rmsCurrent.Outport(1), ...
    productPorts.Inport(2), 'autorouting', 'on');
add_line(inputMetricsPath, productPorts.Outport(1), ...
    pfGuardPorts.Inport(1), 'autorouting', 'on');
add_line(inputMetricsPath, pfGuardPorts.Outport(1), ...
    pfDivide.Inport(2), 'autorouting', 'on');

%% Correct output-current and line-voltage THD
outputMetricsPath = getfullname(sid_handle(mdl, 228));

% The scoring load is purely resistive. Derive its three phase currents
% directly from the reconstructed load phase voltages, avoiding the
% pre-capacitor inductor-current measurement used by the control loop.
loadCurrentGain = add_block('simulink/Math Operations/Gain', ...
    [outputMetricsPath '/Load Current from Voltage'], ...
    'Gain', '1/(Ull_rms/(sqrt(3)*Io_rms))', ...
    'Position', [105 300 175 330]);
phaseVoltage = get_param(sid_handle(mdl, 238), 'PortHandles');
loadCurrentGainPorts = get_param(loadCurrentGain, 'PortHandles');
add_line(outputMetricsPath, phaseVoltage.Outport(1), ...
    loadCurrentGainPorts.Inport(1), 'autorouting', 'on');

meanCurrentPorts = get_param(sid_handle(mdl, 248), 'PortHandles');
powerProductPorts = get_param(sid_handle(mdl, 249), 'PortHandles');
rmsCurrentPorts = get_param(sid_handle(mdl, 252), 'PortHandles');
currentThdPorts = get_param(sid_handle(mdl, 258), 'PortHandles');
loadCurrentDestinations = [ ...
    meanCurrentPorts.Inport(1), ...
    powerProductPorts.Inport(2), ...
    rmsCurrentPorts.Inport(1), ...
    currentThdPorts.Inport(1)];
for k = 1:numel(loadCurrentDestinations)
    disconnect_destination(loadCurrentDestinations(k));
    add_line(outputMetricsPath, loadCurrentGainPorts.Outport(1), ...
        loadCurrentDestinations(k), 'autorouting', 'on');
end

lineVoltageMux = add_block('simulink/Signal Routing/Mux', ...
    [outputMetricsPath '/Line Voltage THD Mux'], ...
    'Inputs', '3', ...
    'Position', [220 125 225 175]);
lineVoltageMuxPorts = get_param(lineVoltageMux, 'PortHandles');
lineVoltageThd = get_param(sid_handle(mdl, 259), 'PortHandles');
disconnect_destination(lineVoltageThd.Inport(1));
lineVoltageSids = [369 372 373];
for k = 1:3
    lineVoltage = get_param(sid_handle(mdl, lineVoltageSids(k)), ...
        'PortHandles');
    add_line(outputMetricsPath, lineVoltage.Outport(1), ...
        lineVoltageMuxPorts.Inport(k), 'autorouting', 'on');
end
add_line(outputMetricsPath, lineVoltageMuxPorts.Outport(1), ...
    lineVoltageThd.Inport(1), 'autorouting', 'on');

% Prevent the output PF display from dividing by zero before the RMS
% estimators have accumulated their first samples.
outputPfDenominatorGuard = add_block( ...
    'simulink/Discontinuities/Saturation', ...
    [outputMetricsPath '/PF Denominator Guard'], ...
    'LowerLimit', 'eps', ...
    'UpperLimit', 'inf', ...
    'Position', [405 335 445 370]);
outputPfDivide = get_param([outputMetricsPath '/Divide2'], ...
    'PortHandles');
outputPfDenominator = get_param([outputMetricsPath '/Sum7'], ...
    'PortHandles');
outputPfGuardPorts = get_param(outputPfDenominatorGuard, ...
    'PortHandles');
disconnect_destination(outputPfDivide.Inport(2));
add_line(outputMetricsPath, outputPfDenominator.Outport(1), ...
    outputPfGuardPorts.Inport(1), 'autorouting', 'on');
add_line(outputMetricsPath, outputPfGuardPorts.Outport(1), ...
    outputPfDivide.Inport(2), 'autorouting', 'on');

%% Add explicit efficiency calculation
unmodelledLoss = add_block('simulink/Sources/Constant', ...
    [mdl '/Unmodelled Loss'], ...
    'Value', 'P_unmodelled_W', ...
    'Position', [2425 905 2480 935]);
pinTotal = add_block('simulink/Math Operations/Sum', ...
    [mdl '/Pin Total'], ...
    'Inputs', '++', ...
    'Position', [2510 860 2540 910]);
efficiencyDenominatorGuard = add_block( ...
    'simulink/Discontinuities/Saturation', ...
    [mdl '/Efficiency Denominator Guard'], ...
    'LowerLimit', 'eps', ...
    'UpperLimit', 'inf', ...
    'Position', [2555 860 2595 910]);
efficiencyDivide = add_block('simulink/Math Operations/Product', ...
    [mdl '/Efficiency Divide'], ...
    'Inputs', '*/', ...
    'Position', [2630 770 2660 820]);
efficiencyPercent = add_block('simulink/Math Operations/Gain', ...
    [mdl '/Efficiency Percent'], ...
    'Gain', '100', ...
    'Position', [2690 780 2745 810]);
efficiencyDisplay = add_block('simulink/Sinks/Display', ...
    [mdl '/Efficiency Display'], ...
    'Position', [2780 775 2850 815]);

inputMetrics = get_param(sid_handle(mdl, 340), 'PortHandles');
outputMetrics = get_param(sid_handle(mdl, 228), 'PortHandles');
lossPorts = get_param(unmodelledLoss, 'PortHandles');
pinTotalPorts = get_param(pinTotal, 'PortHandles');
efficiencyGuardPorts = get_param(efficiencyDenominatorGuard, ...
    'PortHandles');
efficiencyDividePorts = get_param(efficiencyDivide, 'PortHandles');
efficiencyPercentPorts = get_param(efficiencyPercent, 'PortHandles');
efficiencyDisplayPorts = get_param(efficiencyDisplay, 'PortHandles');

add_line(mdl, inputMetrics.Outport(4), pinTotalPorts.Inport(1), ...
    'autorouting', 'on');
add_line(mdl, lossPorts.Outport(1), pinTotalPorts.Inport(2), ...
    'autorouting', 'on');
add_line(mdl, outputMetrics.Outport(3), efficiencyDividePorts.Inport(1), ...
    'autorouting', 'on');
add_line(mdl, pinTotalPorts.Outport(1), efficiencyGuardPorts.Inport(1), ...
    'autorouting', 'on');
add_line(mdl, efficiencyGuardPorts.Outport(1), ...
    efficiencyDividePorts.Inport(2), ...
    'autorouting', 'on');
add_line(mdl, efficiencyDividePorts.Outport(1), ...
    efficiencyPercentPorts.Inport(1), 'autorouting', 'on');
add_line(mdl, efficiencyPercentPorts.Outport(1), ...
    efficiencyDisplayPorts.Inport(1), 'autorouting', 'on');

%% Structural verification
set_param(mdl, 'SimulationCommand', 'update');

assert(strcmp(get_param(sid_handle(mdl, 5), 'Inductance'), 'L_filter'));
assert(strcmp(get_param(sid_handle(mdl, 17), 'Capacitance'), 'C_filter'));
assert(strcmp(get_param(sid_handle(mdl, 43), 'Wr'), '2*pi*fi'));
assert(strcmp(get_param(sid_handle(mdl, 153), 'Wr'), '2*pi*f_out'));
assert(strcmp(get_param(sid_handle(mdl, 182), 'Resistance'), ...
    'Ull_rms/(sqrt(3)*Io_rms)'));
assert(strcmp(get_param(sid_handle(mdl, 353), 'Freq'), 'fi'));

vabcPorts = get_param(sid_handle(mdl, 284), 'PortHandles');
zeroSequencePorts = get_param(sid_handle(mdl, 285), 'PortHandles');
sum2Ports = get_param(sid_handle(mdl, 322), 'PortHandles');
zeroSequenceInputLine = get_param(zeroSequencePorts.Inport(1), 'Line');
sum2ZeroSequenceLine = get_param(sum2Ports.Inport(2), 'Line');
assert(zeroSequenceInputLine ~= -1);
assert(sum2ZeroSequenceLine ~= -1);
assert(get_param(zeroSequenceInputLine, 'SrcPortHandle') == ...
    vabcPorts.Outport(1));
assert(get_param(sum2ZeroSequenceLine, 'SrcPortHandle') == ...
    zeroSequencePorts.Outport(1));

v = [0.6; -0.1; -0.5];
v0 = -0.5 * (max(v) + min(v));
d = 0.5 * (1 + v + v0);
assert(all(d >= 0 & d <= 1));
assert(abs(max(d) + min(d) - 1) < 1e-12);

save_system(mdl, dstFile);
fprintf('Created and structurally verified: %s\n', dstFile);
fprintf('L_filter = 1.1 mH, C_filter = 10 uF, DC link remains 4.4 mF.\n');
end

function h = sid_handle(mdl, sid)
h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
assert(h ~= -1, 'SID %d not found in %s.', sid, mdl);
end

function set_sid(mdl, sid, varargin)
set_param(sid_handle(mdl, sid), varargin{:});
end

function disconnect_destination(portHandle)
disconnect_port_line(portHandle);
end

function disconnect_port_line(portHandle)
lineHandle = get_param(portHandle, 'Line');
if lineHandle ~= -1
    delete_line(lineHandle);
end
end

function close_if_loaded(mdl)
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
end
