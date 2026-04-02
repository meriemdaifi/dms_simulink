%% DMS_ADAS_PROJECT - Programmatically build and run a full DMS Simulink model
%
%  This script creates a complete Driver Monitoring System (DMS) ADAS
%  Simulink model from scratch.  It does NOT require any pre-existing .slx
%  file; every block, line, subsystem, and parameter is created in code.
%
%  Model Architecture
%  ------------------
%  1. Sensor Input Subsystem
%     - Simulated camera signals (PERCLOS, blink duration, yawn flag)
%     - Gaze angle (horizontal / vertical)
%     - Head pose (yaw, pitch)
%     - Vehicle signals (speed, steering torque, lateral deviation)
%
%  2. Driver State Estimation Subsystem
%     - Drowsiness estimator  (weighted fusion)
%     - Distraction detector  (gaze-off + head-turn logic)
%     - Fatigue estimator     (steering entropy + lane deviation + duration)
%     - Overall risk score    (max of the three scores)
%
%  3. Alert & Intervention Subsystem
%     - Multi-level alert generator  (0-None … 4-Critical)
%     - Visual / Audio / Haptic output arbitration
%     - Lane-keeping-assist torque overlay
%     - Controlled speed reduction & emergency stop
%
%  4. Dashboard & Logging Subsystem
%     - Scope displays for live monitoring
%     - To-Workspace blocks for post-simulation analysis
%
%  Usage
%  -----
%     >> dms_config          % load parameters
%     >> dms_adas_project    % build, simulate, and analyse
%
%  Copyright (c) 2026 DMS ADAS Project

%% ========================================================================
%  0.  Initialisation
%  ========================================================================
fprintf('=== DMS ADAS Project - Simulink Model Builder ===\n');

% Load configuration (creates Config struct & workspace variables)
dms_config;

modelName = 'DMS_ADAS_Model';

% Close & delete any previous version
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

% Create a new blank model
new_system(modelName);
open_system(modelName);

% Set solver and simulation parameters
set_param(modelName, ...
    'StopTime',            num2str(Config.Simulation.StopTime), ...
    'SolverType',          'Fixed-step', ...
    'Solver',              'FixedStepDiscrete', ...
    'FixedStep',           Config.Simulation.FixedStep, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput',          'on', ...
    'SaveTime',            'on');

fprintf('  [1/5] Created blank model "%s".\n', modelName);

%% ========================================================================
%  1.  Sensor Input Subsystem
%  ========================================================================
sensorSys = [modelName '/Sensor_Inputs'];
add_block('simulink/Ports & Subsystems/Subsystem', sensorSys);
% Remove default In1/Out1 inside the subsystem
delete_line(sensorSys, 'In1/1', 'Out1/1');
delete_block([sensorSys '/In1']);
delete_block([sensorSys '/Out1']);

% ---- Helper: add a sensor signal generator inside sensorSys -----------
%  Each sensor is a Sine Wave + Bias + Gain + Saturation → Outport
nextY = 30;  % vertical spacing
sensorSignals = { ...
%   Name               Bias   Amp    Freq(rad/s) Phase  Min  Max
    'PERCLOS',          0.25,  0.25,  0.05,       0,     0,   1;
    'BlinkDuration',    0.30,  0.25,  0.10,       1,     0,   2;
    'YawnFlag',         0.20,  0.30,  0.03,       2,     0,   1;
    'GazeAngle',        0,     35,    0.20,       0,    -90,  90;
    'HeadYaw',          0,     30,    0.15,       0.5,  -90,  90;
    'HeadPitch',        0,     20,    0.08,       1.0,  -45,  45;
    'VehicleSpeed',     80,    20,    0.02,       0,     0,   200;
    'SteeringTorque',   0,     5,     0.30,       0,    -20,  20;
    'LateralDeviation', 0,     0.5,   0.25,       0.3,  -3,   3;
    };

numSensors = size(sensorSignals, 1);
for k = 1:numSensors
    sigName  = sensorSignals{k, 1};
    sigBias  = sensorSignals{k, 2};
    sigAmp   = sensorSignals{k, 3};
    sigFreq  = sensorSignals{k, 4};
    sigPhase = sensorSignals{k, 5};
    sigMin   = sensorSignals{k, 6};
    sigMax   = sensorSignals{k, 7};

    xBase = 50;
    yBase = nextY;

    % Sine Wave
    sinBlk = [sensorSys '/' sigName '_Sine'];
    add_block('simulink/Sources/Sine Wave', sinBlk, ...
        'Amplitude',  num2str(sigAmp), ...
        'Frequency',  num2str(sigFreq), ...
        'Phase',      num2str(sigPhase), ...
        'SampleTime', 'Ts', ...
        'Position',   [xBase yBase xBase+60 yBase+30]);

    % Bias (Constant + Sum)
    biasBlk = [sensorSys '/' sigName '_Bias'];
    add_block('simulink/Math Operations/Add', biasBlk, ...
        'Inputs',   '++', ...
        'Position', [xBase+100 yBase xBase+130 yBase+30]);

    biasConst = [sensorSys '/' sigName '_BiasVal'];
    add_block('simulink/Sources/Constant', biasConst, ...
        'Value',    num2str(sigBias), ...
        'Position', [xBase+50 yBase+50 xBase+80 yBase+70]);

    % Saturation
    satBlk = [sensorSys '/' sigName '_Sat'];
    add_block('simulink/Discontinuities/Saturation', satBlk, ...
        'UpperLimit', num2str(sigMax), ...
        'LowerLimit', num2str(sigMin), ...
        'Position',   [xBase+170 yBase xBase+210 yBase+30]);

    % Outport
    outBlk = [sensorSys '/' sigName];
    add_block('simulink/Sinks/Out1', outBlk, ...
        'Port', num2str(k), ...
        'Position', [xBase+260 yBase xBase+280 yBase+20]);

    % Wiring
    add_line(sensorSys, [sigName '_Sine/1'],    [sigName '_Bias/1']);
    add_line(sensorSys, [sigName '_BiasVal/1'], [sigName '_Bias/2']);
    add_line(sensorSys, [sigName '_Bias/1'],    [sigName '_Sat/1']);
    add_line(sensorSys, [sigName '_Sat/1'],     [sigName '/1']);

    nextY = nextY + 100;
end

% Position the Sensor subsystem in the top-level model
set_param(sensorSys, 'Position', [80, 60, 250, 60 + numSensors*30]);

fprintf('  [2/5] Built Sensor Input Subsystem (%d signals).\n', numSensors);

%% ========================================================================
%  2.  Driver State Estimation Subsystem
%  ========================================================================
stateSys = [modelName '/Driver_State_Estimation'];
add_block('simulink/Ports & Subsystems/Subsystem', stateSys);
delete_line(stateSys, 'In1/1', 'Out1/1');
delete_block([stateSys '/In1']);
delete_block([stateSys '/Out1']);

% --- Inports (match sensor outputs) ---
sensorNames = sensorSignals(:,1)';
for k = 1:numSensors
    add_block('simulink/Sources/In1', [stateSys '/' sensorNames{k}], ...
        'Port', num2str(k), ...
        'Position', [30, 30+(k-1)*60, 60, 50+(k-1)*60]);
end

% === 2a. Drowsiness Estimator (MATLAB Fcn) ==============================
drowsyFcn = [stateSys '/Drowsiness_Estimator'];
add_block('simulink/User-Defined Functions/MATLAB Function', drowsyFcn, ...
    'Position', [250, 30, 420, 110]);

% Write the MATLAB function code
drowsyScript = { ...
    'function score = DrowsinessEstimator(perclos, blinkDur, headPitch, yawnFlag)' ...
    '%#codegen' ...
    'persistent w pt bt ht yt' ...
    'if isempty(w)' ...
    '    w  = [0.40, 0.25, 0.20, 0.15];' ...
    '    pt = 0.4;  bt = 0.5;  ht = 15;  yt = 0.5;' ...
    'end' ...
    's1 = min(perclos  / pt, 1);' ...
    's2 = min(blinkDur / bt, 1);' ...
    's3 = min(abs(headPitch) / ht, 1);' ...
    's4 = min(yawnFlag / yt, 1);' ...
    'score = max(0, min(1, w(1)*s1 + w(2)*s2 + w(3)*s3 + w(4)*s4));' ...
    };
sf = sfroot;
% Defer MATLAB Function script update to after model is fully built

% === 2b. Distraction Detector (MATLAB Fcn) ==============================
distractFcn = [stateSys '/Distraction_Detector'];
add_block('simulink/User-Defined Functions/MATLAB Function', distractFcn, ...
    'Position', [250, 140, 420, 220]);

% === 2c. Fatigue Estimator (MATLAB Fcn) =================================
fatigueFcn = [stateSys '/Fatigue_Estimator'];
add_block('simulink/User-Defined Functions/MATLAB Function', fatigueFcn, ...
    'Position', [250, 260, 420, 350]);

% === 2d. Risk Fusion (MinMax -> max of 3 scores) ========================
riskFusion = [stateSys '/Risk_Fusion'];
add_block('simulink/Math Operations/MinMax', riskFusion, ...
    'Function', 'max', ...
    'Inputs',   '3', ...
    'Position', [500, 130, 540, 210]);

% === Outports ============================================================
outNames = {'DrowsinessScore','DistractionScore','FatigueScore','OverallRisk'};
for k = 1:length(outNames)
    add_block('simulink/Sinks/Out1', [stateSys '/' outNames{k}], ...
        'Port', num2str(k), ...
        'Position', [620, 30+(k-1)*70, 650, 50+(k-1)*70]);
end

% --- Wiring inside Driver_State_Estimation --------------------------------
% Drowsiness inputs: PERCLOS(1), BlinkDuration(2), HeadPitch(6), YawnFlag(3)
add_line(stateSys, 'PERCLOS/1',       'Drowsiness_Estimator/1');
add_line(stateSys, 'BlinkDuration/1', 'Drowsiness_Estimator/2');
add_line(stateSys, 'HeadPitch/1',     'Drowsiness_Estimator/3');
add_line(stateSys, 'YawnFlag/1',      'Drowsiness_Estimator/4');

% Distraction inputs: GazeAngle(4), HeadYaw(5)
add_line(stateSys, 'GazeAngle/1', 'Distraction_Detector/1');
add_line(stateSys, 'HeadYaw/1',   'Distraction_Detector/2');

% Fatigue inputs: SteeringTorque(8), LateralDeviation(9), VehicleSpeed(7)
add_line(stateSys, 'SteeringTorque/1',  'Fatigue_Estimator/1');
add_line(stateSys, 'LateralDeviation/1','Fatigue_Estimator/2');
add_line(stateSys, 'VehicleSpeed/1',    'Fatigue_Estimator/3');

% Outputs to risk fusion
add_line(stateSys, 'Drowsiness_Estimator/1', 'Risk_Fusion/1');
add_line(stateSys, 'Distraction_Detector/1', 'Risk_Fusion/2');
add_line(stateSys, 'Fatigue_Estimator/1',    'Risk_Fusion/3');

% Outputs to outports
add_line(stateSys, 'Drowsiness_Estimator/1', 'DrowsinessScore/1');
add_line(stateSys, 'Distraction_Detector/1', 'DistractionScore/1');
add_line(stateSys, 'Fatigue_Estimator/1',    'FatigueScore/1');
add_line(stateSys, 'Risk_Fusion/1',          'OverallRisk/1');

% Position in top-level model
set_param(stateSys, 'Position', [350, 60, 560, 60 + numSensors*30]);

fprintf('  [3/5] Built Driver State Estimation Subsystem.\n');

%% ========================================================================
%  3.  Alert & Intervention Subsystem
%  ========================================================================
alertSys = [modelName '/Alert_Intervention'];
add_block('simulink/Ports & Subsystems/Subsystem', alertSys);
delete_line(alertSys, 'In1/1', 'Out1/1');
delete_block([alertSys '/In1']);
delete_block([alertSys '/Out1']);

% Inports
alertInNames = {'DrowsinessScore','DistractionScore','FatigueScore','OverallRisk','VehicleSpeed','LateralDeviation'};
for k = 1:length(alertInNames)
    add_block('simulink/Sources/In1', [alertSys '/' alertInNames{k}], ...
        'Port', num2str(k), ...
        'Position', [30, 30+(k-1)*60, 60, 50+(k-1)*60]);
end

% --- Alert Level Generator (MATLAB Fcn) ---
alertGenBlk = [alertSys '/Alert_Level_Generator'];
add_block('simulink/User-Defined Functions/MATLAB Function', alertGenBlk, ...
    'Position', [200, 30, 380, 100]);

% --- Visual Alert (Gain -> Saturation) ---
add_block('simulink/Math Operations/Gain', [alertSys '/Visual_Gain'], ...
    'Gain', '1', 'Position', [450, 30, 490, 50]);
add_block('simulink/Discontinuities/Saturation', [alertSys '/Visual_Sat'], ...
    'UpperLimit', '4', 'LowerLimit', '0', ...
    'Position', [530, 30, 570, 50]);

% --- Audio Alert (Gain -> Saturation) ---
add_block('simulink/Math Operations/Gain', [alertSys '/Audio_Gain'], ...
    'Gain', '1', 'Position', [450, 80, 490, 100]);
add_block('simulink/Discontinuities/Saturation', [alertSys '/Audio_Sat'], ...
    'UpperLimit', '4', 'LowerLimit', '0', ...
    'Position', [530, 80, 570, 100]);

% --- Haptic Alert (Gain -> Saturation) ---
add_block('simulink/Math Operations/Gain', [alertSys '/Haptic_Gain'], ...
    'Gain', '1', 'Position', [450, 130, 490, 150]);
add_block('simulink/Discontinuities/Saturation', [alertSys '/Haptic_Sat'], ...
    'UpperLimit', '4', 'LowerLimit', '0', ...
    'Position', [530, 130, 570, 150]);

% --- Lane Keeping Assist (MATLAB Fcn) ---
lkaBlk = [alertSys '/LKA_Controller'];
add_block('simulink/User-Defined Functions/MATLAB Function', lkaBlk, ...
    'Position', [200, 200, 380, 270]);

% --- Speed Reduction Controller (MATLAB Fcn) ---
speedCtrlBlk = [alertSys '/Speed_Controller'];
add_block('simulink/User-Defined Functions/MATLAB Function', speedCtrlBlk, ...
    'Position', [200, 300, 380, 370]);

% Outports
alertOutNames = {'AlertLevel','VisualAlert','AudioAlert','HapticAlert', ...
                 'LKA_Torque','SpeedCommand'};
for k = 1:length(alertOutNames)
    add_block('simulink/Sinks/Out1', [alertSys '/' alertOutNames{k}], ...
        'Port', num2str(k), ...
        'Position', [660, 30+(k-1)*60, 690, 50+(k-1)*60]);
end

% Wiring inside Alert subsystem
add_line(alertSys, 'OverallRisk/1',        'Alert_Level_Generator/1');
add_line(alertSys, 'Alert_Level_Generator/1', 'AlertLevel/1');
add_line(alertSys, 'Alert_Level_Generator/1', 'Visual_Gain/1');
add_line(alertSys, 'Visual_Gain/1',           'Visual_Sat/1');
add_line(alertSys, 'Visual_Sat/1',            'VisualAlert/1');
add_line(alertSys, 'Alert_Level_Generator/1', 'Audio_Gain/1');
add_line(alertSys, 'Audio_Gain/1',            'Audio_Sat/1');
add_line(alertSys, 'Audio_Sat/1',             'AudioAlert/1');
add_line(alertSys, 'Alert_Level_Generator/1', 'Haptic_Gain/1');
add_line(alertSys, 'Haptic_Gain/1',           'Haptic_Sat/1');
add_line(alertSys, 'Haptic_Sat/1',            'HapticAlert/1');

% LKA: inputs are LateralDeviation and VehicleSpeed
add_line(alertSys, 'LateralDeviation/1', 'LKA_Controller/1');
add_line(alertSys, 'VehicleSpeed/1',     'LKA_Controller/2');
add_line(alertSys, 'LKA_Controller/1',   'LKA_Torque/1');

% Speed controller: inputs are OverallRisk and VehicleSpeed
add_line(alertSys, 'OverallRisk/1',       'Speed_Controller/1');
add_line(alertSys, 'VehicleSpeed/1',      'Speed_Controller/2');
add_line(alertSys, 'Speed_Controller/1',  'SpeedCommand/1');

% Position in top-level model
set_param(alertSys, 'Position', [660, 60, 870, 60 + numSensors*30]);

fprintf('  [4/5] Built Alert & Intervention Subsystem.\n');

%% ========================================================================
%  4.  Dashboard & Logging Subsystem
%  ========================================================================
dashSys = [modelName '/Dashboard_Logging'];
add_block('simulink/Ports & Subsystems/Subsystem', dashSys);
delete_line(dashSys, 'In1/1', 'Out1/1');
delete_block([dashSys '/In1']);
delete_block([dashSys '/Out1']);

dashInNames = {'DrowsinessScore','DistractionScore','FatigueScore', ...
               'OverallRisk','AlertLevel','VehicleSpeed','LKA_Torque','SpeedCommand'};
for k = 1:length(dashInNames)
    add_block('simulink/Sources/In1', [dashSys '/' dashInNames{k}], ...
        'Port', num2str(k), ...
        'Position', [30, 30+(k-1)*50, 60, 50+(k-1)*50]);
end

% To Workspace blocks for logging
for k = 1:length(dashInNames)
    twBlk = [dashSys '/Log_' dashInNames{k}];
    add_block('simulink/Sinks/To Workspace', twBlk, ...
        'VariableName', ['log_' dashInNames{k}], ...
        'MaxDataPoints', num2str(Config.Logging.MaxPoints), ...
        'Decimation',    num2str(Config.Logging.Decimation), ...
        'SaveFormat',    'Timeseries', ...
        'Position',      [200, 25+(k-1)*50, 320, 45+(k-1)*50]);
    add_line(dashSys, [dashInNames{k} '/1'], ['Log_' dashInNames{k} '/1']);
end

% Scope: Driver State Scores
add_block('simulink/Sinks/Scope', [dashSys '/Scope_DriverState'], ...
    'NumInputPorts', '3', ...
    'Position', [400, 30, 440, 110]);
add_line(dashSys, 'DrowsinessScore/1',  'Scope_DriverState/1');
add_line(dashSys, 'DistractionScore/1', 'Scope_DriverState/2');
add_line(dashSys, 'FatigueScore/1',     'Scope_DriverState/3');

% Scope: Alert & Intervention
add_block('simulink/Sinks/Scope', [dashSys '/Scope_Alerts'], ...
    'NumInputPorts', '3', ...
    'Position', [400, 140, 440, 220]);
add_line(dashSys, 'AlertLevel/1',   'Scope_Alerts/1');
add_line(dashSys, 'LKA_Torque/1',   'Scope_Alerts/2');
add_line(dashSys, 'SpeedCommand/1', 'Scope_Alerts/3');

% Position in top-level model
set_param(dashSys, 'Position', [970, 60, 1150, 60 + numSensors*30]);

fprintf('  [4/5] Built Dashboard & Logging Subsystem.\n');

%% ========================================================================
%  5.  Top-Level Wiring
%  ========================================================================

% Sensor → Driver State Estimation (all 9 ports)
for k = 1:numSensors
    add_line(modelName, ['Sensor_Inputs/' num2str(k)], ...
                        ['Driver_State_Estimation/' num2str(k)], ...
                        'autorouting', 'smart');
end

% Driver State Estimation → Alert Subsystem
%  Outputs: DrowsinessScore(1), DistractionScore(2), FatigueScore(3), OverallRisk(4)
%  Alert Inports: DrowsinessScore(1), DistractionScore(2), FatigueScore(3),
%                 OverallRisk(4), VehicleSpeed(5), LateralDeviation(6)
for k = 1:4
    add_line(modelName, ['Driver_State_Estimation/' num2str(k)], ...
                        ['Alert_Intervention/' num2str(k)], ...
                        'autorouting', 'smart');
end
% VehicleSpeed (sensor port 7) -> Alert port 5
add_line(modelName, 'Sensor_Inputs/7', 'Alert_Intervention/5', 'autorouting', 'smart');
% LateralDeviation (sensor port 9) -> Alert port 6
add_line(modelName, 'Sensor_Inputs/9', 'Alert_Intervention/6', 'autorouting', 'smart');

% Driver State + Alert → Dashboard
% Dashboard ports:  1-DrowsinessScore, 2-DistractionScore, 3-FatigueScore,
%                   4-OverallRisk, 5-AlertLevel, 6-VehicleSpeed,
%                   7-LKA_Torque, 8-SpeedCommand
for k = 1:4
    add_line(modelName, ['Driver_State_Estimation/' num2str(k)], ...
                        ['Dashboard_Logging/' num2str(k)], ...
                        'autorouting', 'smart');
end
% AlertLevel (Alert out 1) -> Dashboard 5
add_line(modelName, 'Alert_Intervention/1', 'Dashboard_Logging/5', 'autorouting', 'smart');
% VehicleSpeed (sensor 7) -> Dashboard 6
add_line(modelName, 'Sensor_Inputs/7', 'Dashboard_Logging/6', 'autorouting', 'smart');
% LKA_Torque (Alert out 5) -> Dashboard 7
add_line(modelName, 'Alert_Intervention/5', 'Dashboard_Logging/7', 'autorouting', 'smart');
% SpeedCommand (Alert out 6) -> Dashboard 8
add_line(modelName, 'Alert_Intervention/6', 'Dashboard_Logging/8', 'autorouting', 'smart');

fprintf('  [5/5] Top-level wiring complete.\n');

%% ========================================================================
%  6.  Update MATLAB Function Block Scripts
%  ========================================================================
%  Simulink's MATLAB Function blocks require their script to be set via the
%  Stateflow API after the model is fully constructed.

% --- Helper to set MATLAB Function script --------------------------------
setMatlabFcnScript = @(blkPath, script) setMATLABFunctionScript(blkPath, script);

% Drowsiness Estimator
drowsyCode = sprintf([...
    'function score = DrowsinessEstimator(perclos, blinkDur, headPitch, yawnFlag)\n' ...
    '%%#codegen\n' ...
    'persistent w pt bt ht yt\n' ...
    'if isempty(w)\n' ...
    '    w  = [0.40, 0.25, 0.20, 0.15];\n' ...
    '    pt = 0.4;  bt = 0.5;  ht = 15;  yt = 0.5;\n' ...
    'end\n' ...
    's1 = min(perclos  / pt, 1);\n' ...
    's2 = min(blinkDur / bt, 1);\n' ...
    's3 = min(abs(headPitch) / ht, 1);\n' ...
    's4 = min(yawnFlag / yt, 1);\n' ...
    'score = max(0, min(1, w(1)*s1 + w(2)*s2 + w(3)*s3 + w(4)*s4));\n' ...
    ]);

% Distraction Detector
distractCode = sprintf([...
    'function score = DistractionDetector(gazeAngle, headYaw)\n' ...
    '%%#codegen\n' ...
    'persistent gazeThresh headThresh\n' ...
    'if isempty(gazeThresh)\n' ...
    '    gazeThresh = 30;\n' ...
    '    headThresh = 25;\n' ...
    'end\n' ...
    'gazeOff = min(abs(gazeAngle) / gazeThresh, 1);\n' ...
    'headOff = min(abs(headYaw)   / headThresh, 1);\n' ...
    'score   = max(0, min(1, 0.6*gazeOff + 0.4*headOff));\n' ...
    ]);

% Fatigue Estimator
fatigueCode = sprintf([...
    'function score = FatigueEstimator(steeringTorque, lateralDev, vehicleSpeed)\n' ...
    '%%#codegen\n' ...
    'persistent stThresh ldThresh\n' ...
    'if isempty(stThresh)\n' ...
    '    stThresh = 10;\n' ...
    '    ldThresh = 1.5;\n' ...
    'end\n' ...
    'stScore = min(abs(steeringTorque) / stThresh, 1);\n' ...
    'ldScore = min(abs(lateralDev)     / ldThresh, 1);\n' ...
    'spFactor = max(0, min(vehicleSpeed / 120, 1));\n' ...
    'score = max(0, min(1, (0.4*stScore + 0.4*ldScore) * (0.5 + 0.5*spFactor)));\n' ...
    ]);

% Alert Level Generator
alertCode = sprintf([...
    'function level = AlertLevelGenerator(overallRisk)\n' ...
    '%%#codegen\n' ...
    'if overallRisk >= 0.9\n' ...
    '    level = 4;\n' ...
    'elseif overallRisk >= 0.7\n' ...
    '    level = 3;\n' ...
    'elseif overallRisk >= 0.5\n' ...
    '    level = 2;\n' ...
    'elseif overallRisk >= 0.3\n' ...
    '    level = 1;\n' ...
    'else\n' ...
    '    level = 0;\n' ...
    'end\n' ...
    ]);

% LKA Controller
lkaCode = sprintf([...
    'function torque = LKA_Controller(lateralDev, vehicleSpeed)\n' ...
    '%%#codegen\n' ...
    'persistent gain maxTorque\n' ...
    'if isempty(gain)\n' ...
    '    gain = 0.5;\n' ...
    '    maxTorque = 10;\n' ...
    'end\n' ...
    'speedFactor = max(0.1, min(vehicleSpeed / 100, 1));\n' ...
    'rawTorque   = -gain * lateralDev * speedFactor;\n' ...
    'torque      = max(-maxTorque, min(maxTorque, rawTorque));\n' ...
    ]);

% Speed Controller
speedCode = sprintf([...
    'function command = SpeedController(overallRisk, vehicleSpeed)\n' ...
    '%%#codegen\n' ...
    'persistent maxDecel emergDecel\n' ...
    'if isempty(maxDecel)\n' ...
    '    maxDecel   = 2.0;\n' ...
    '    emergDecel = 5.0;\n' ...
    'end\n' ...
    'if overallRisk >= 0.9\n' ...
    '    command = max(0, vehicleSpeed - emergDecel);\n' ...
    'elseif overallRisk >= 0.7\n' ...
    '    command = max(0, vehicleSpeed - maxDecel);\n' ...
    'elseif overallRisk >= 0.5\n' ...
    '    command = max(0, vehicleSpeed - 0.5*maxDecel);\n' ...
    'else\n' ...
    '    command = vehicleSpeed;\n' ...
    'end\n' ...
    ]);

% Apply scripts using Stateflow root
try
    rt = sfroot;
    mdl = rt.find('-isa', 'Simulink.BlockDiagram', '-and', 'Name', modelName);
    charts = mdl.find('-isa', 'Stateflow.EMChart');

    for c = 1:length(charts)
        chartPath = charts(c).Path;
        if contains(chartPath, 'Drowsiness_Estimator')
            charts(c).Script = drowsyCode;
        elseif contains(chartPath, 'Distraction_Detector')
            charts(c).Script = distractCode;
        elseif contains(chartPath, 'Fatigue_Estimator')
            charts(c).Script = fatigueCode;
        elseif contains(chartPath, 'Alert_Level_Generator')
            charts(c).Script = alertCode;
        elseif contains(chartPath, 'LKA_Controller')
            charts(c).Script = lkaCode;
        elseif contains(chartPath, 'Speed_Controller')
            charts(c).Script = speedCode;
        end
    end
    fprintf('  MATLAB Function scripts applied via Stateflow API.\n');
catch ME
    warning('DMS:ScriptUpdate', ...
        'Could not update MATLAB Function scripts: %s\nManual update may be required.', ME.message);
end

%% ========================================================================
%  7.  Model Annotations
%  ========================================================================
try
    add_block('simulink/Model-Wide Utilities/Model Info', [modelName '/ModelInfo'], ...
        'Position', [80, 400, 300, 450]);
catch
    % Model Info block may not be available in all versions
end

% Add annotation
annotation = Simulink.Annotation([modelName '/Title']);
annotation.Text = sprintf('DMS (Driver Monitoring System) ADAS Model\nAuto-generated by dms_adas_project.m');
annotation.Position = [500, 10];

%% ========================================================================
%  8.  Save the Model
%  ========================================================================
save_system(modelName);
fprintf('  Model saved as "%s.slx".\n', modelName);

%% ========================================================================
%  9.  Simulate
%  ========================================================================
fprintf('\n--- Running simulation (%.0f seconds, Ts=%.3f s) ---\n', ...
    Config.Simulation.StopTime, Config.Simulation.SampleTime);

simOut = sim(modelName, ...
    'StopTime',   num2str(Config.Simulation.StopTime), ...
    'ReturnWorkspaceOutputs', 'on');

fprintf('  Simulation complete.  Elapsed wall-clock: %.1f s\n', simOut.SimulationMetadata.TimingInfo.TotalElapsedWallTime);

%% ========================================================================
% 10.  Post-Processing & Visualisation
%  ========================================================================
fprintf('\n--- Post-Processing Results ---\n');

% Extract logged signals
try
    t  = simOut.tout;

    drowsy   = evalin('base', 'log_DrowsinessScore');
    distract = evalin('base', 'log_DistractionScore');
    fatigue  = evalin('base', 'log_FatigueScore');
    risk     = evalin('base', 'log_OverallRisk');
    alertLvl = evalin('base', 'log_AlertLevel');
    vSpeed   = evalin('base', 'log_VehicleSpeed');
    lkaTrq   = evalin('base', 'log_LKA_Torque');
    spdCmd   = evalin('base', 'log_SpeedCommand');

    % --- Figure 1: Driver State Scores ---
    figure('Name', 'DMS - Driver State Scores', 'NumberTitle', 'off');
    subplot(3,1,1);
    plot(drowsy.Time, drowsy.Data, 'r', 'LineWidth', 1.2);
    ylabel('Drowsiness'); title('Driver State Scores'); grid on;
    ylim([0 1]);

    subplot(3,1,2);
    plot(distract.Time, distract.Data, 'b', 'LineWidth', 1.2);
    ylabel('Distraction'); grid on;
    ylim([0 1]);

    subplot(3,1,3);
    plot(fatigue.Time, fatigue.Data, 'm', 'LineWidth', 1.2);
    ylabel('Fatigue'); xlabel('Time (s)'); grid on;
    ylim([0 1]);

    % --- Figure 2: Alert Level & Risk ---
    figure('Name', 'DMS - Alert & Risk', 'NumberTitle', 'off');
    subplot(2,1,1);
    plot(risk.Time, risk.Data, 'k', 'LineWidth', 1.5);
    ylabel('Overall Risk'); title('Risk & Alert Level'); grid on;
    ylim([0 1]);

    subplot(2,1,2);
    stairs(alertLvl.Time, alertLvl.Data, 'r', 'LineWidth', 1.5);
    ylabel('Alert Level (0-4)'); xlabel('Time (s)'); grid on;
    ylim([-0.5 4.5]);

    % --- Figure 3: Vehicle Intervention ---
    figure('Name', 'DMS - Vehicle Intervention', 'NumberTitle', 'off');
    subplot(3,1,1);
    plot(vSpeed.Time, vSpeed.Data, 'b', 'LineWidth', 1.2);
    ylabel('Speed (km/h)'); title('Vehicle Intervention'); grid on;

    subplot(3,1,2);
    plot(lkaTrq.Time, lkaTrq.Data, 'g', 'LineWidth', 1.2);
    ylabel('LKA Torque (Nm)'); grid on;

    subplot(3,1,3);
    plot(spdCmd.Time, spdCmd.Data, 'Color', [0.8 0.4 0], 'LineWidth', 1.2);
    ylabel('Speed Cmd (km/h)'); xlabel('Time (s)'); grid on;

    fprintf('  Figures generated successfully.\n');
catch ME
    warning('DMS:PostProcess', 'Post-processing error: %s', ME.message);
end

%% ========================================================================
% 11.  Summary Statistics
%  ========================================================================
try
    fprintf('\n========== DMS Simulation Summary ==========\n');
    fprintf('  Simulation time     : 0 – %.0f s\n', Config.Simulation.StopTime);
    fprintf('  Sample time         : %.3f s (%.0f Hz)\n', ...
        Config.Simulation.SampleTime, 1/Config.Simulation.SampleTime);
    fprintf('  Mean Drowsiness     : %.3f\n', mean(drowsy.Data));
    fprintf('  Mean Distraction    : %.3f\n', mean(distract.Data));
    fprintf('  Mean Fatigue        : %.3f\n', mean(fatigue.Data));
    fprintf('  Peak Overall Risk   : %.3f\n', max(risk.Data));
    fprintf('  Max Alert Level     : %d\n',   max(alertLvl.Data));
    fprintf('  Mean Vehicle Speed  : %.1f km/h\n', mean(vSpeed.Data));
    fprintf('=============================================\n');
catch
    % Silently skip if logs unavailable
end

fprintf('\nDMS ADAS Project complete.\n');

%% ========================================================================
%  Local Functions
%  ========================================================================

function setMATLABFunctionScript(~, ~)
    % Placeholder - actual script setting is done via sfroot above
end
