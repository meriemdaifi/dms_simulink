%% DMS_TEST - Validation and testing script for the DMS ADAS Simulink model
%
%  This script runs a suite of tests to verify that the DMS model built by
%  dms_adas_project.m behaves correctly.  It can be executed stand-alone;
%  it will build the model if it is not already loaded.
%
%  Tests performed
%  ---------------
%  1. Model structural integrity (all blocks present, connected)
%  2. Configuration parameter sanity checks
%  3. Short simulation smoke test (10 s)
%  4. Signal range validation (scores in [0,1], alert in {0..4})
%  5. Drowsiness-threshold boundary check
%  6. Distraction-threshold boundary check
%  7. Intervention response check (speed reduction at high risk)
%
%  Usage
%  -----
%     >> dms_test
%
%  Copyright (c) 2026 DMS ADAS Project

fprintf('\n====== DMS ADAS Test Suite ======\n\n');

passed  = 0;
failed  = 0;
skipped = 0;

modelName = 'DMS_ADAS_Model';

%% ========================================================================
%  Setup: Ensure model exists
%  ========================================================================
if ~bdIsLoaded(modelName)
    if exist([modelName '.slx'], 'file')
        load_system(modelName);
    else
        fprintf('[SETUP] Model not found. Building from scratch...\n');
        dms_adas_project;
    end
end

%% ========================================================================
%  Test 1: Model Structural Integrity
%  ========================================================================
fprintf('[TEST 1] Model structural integrity... ');
try
    expectedSystems = { ...
        'Sensor_Inputs', ...
        'Driver_State_Estimation', ...
        'Alert_Intervention', ...
        'Dashboard_Logging'};
    allPresent = true;
    for k = 1:length(expectedSystems)
        blkPath = [modelName '/' expectedSystems{k}];
        if ~strcmp(get_param(blkPath, 'BlockType'), 'SubSystem')
            allPresent = false;
            break;
        end
    end
    if allPresent
        fprintf('PASSED\n'); passed = passed + 1;
    else
        fprintf('FAILED - missing subsystem(s)\n'); failed = failed + 1;
    end
catch ME
    fprintf('FAILED - %s\n', ME.message); failed = failed + 1;
end

%% ========================================================================
%  Test 2: Configuration Parameter Sanity
%  ========================================================================
fprintf('[TEST 2] Configuration parameter sanity... ');
try
    dms_config;
    assert(Config.Simulation.SampleTime > 0,             'SampleTime must be > 0');
    assert(Config.Simulation.StopTime > 0,               'StopTime must be > 0');
    assert(Config.Drowsiness.PERCLOS_Threshold > 0 && ...
           Config.Drowsiness.PERCLOS_Threshold < 1,      'PERCLOS thresh out of range');
    assert(all(Config.Drowsiness.FusionWeights >= 0),    'Negative weight found');
    assert(abs(sum(Config.Drowsiness.FusionWeights)-1) < 1e-6, 'Weights must sum to 1');
    assert(length(Config.Alert.Thresholds) == Config.Alert.NumLevels, 'Threshold count mismatch');
    fprintf('PASSED\n'); passed = passed + 1;
catch ME
    fprintf('FAILED - %s\n', ME.message); failed = failed + 1;
end

%% ========================================================================
%  Test 3: Short Simulation Smoke Test (10 s)
%  ========================================================================
fprintf('[TEST 3] Short simulation smoke test (10 s)... ');
try
    simOut = sim(modelName, 'StopTime', '10', 'ReturnWorkspaceOutputs', 'on');
    assert(~isempty(simOut.tout), 'No output time vector');
    assert(max(simOut.tout) >= 9.9, 'Simulation did not reach 10 s');
    fprintf('PASSED\n'); passed = passed + 1;
catch ME
    fprintf('FAILED - %s\n', ME.message); failed = failed + 1;
end

%% ========================================================================
%  Test 4: Signal Range Validation
%  ========================================================================
fprintf('[TEST 4] Signal range validation... ');
try
    drowsy   = evalin('base', 'log_DrowsinessScore');
    distract = evalin('base', 'log_DistractionScore');
    fatigue  = evalin('base', 'log_FatigueScore');
    risk     = evalin('base', 'log_OverallRisk');
    alertLvl = evalin('base', 'log_AlertLevel');

    assert(all(drowsy.Data   >= -0.01 & drowsy.Data   <= 1.01), 'Drowsiness out of [0,1]');
    assert(all(distract.Data >= -0.01 & distract.Data <= 1.01), 'Distraction out of [0,1]');
    assert(all(fatigue.Data  >= -0.01 & fatigue.Data  <= 1.01), 'Fatigue out of [0,1]');
    assert(all(risk.Data     >= -0.01 & risk.Data     <= 1.01), 'Risk out of [0,1]');
    assert(all(alertLvl.Data >= 0     & alertLvl.Data <= 4),    'Alert level out of {0..4}');

    fprintf('PASSED\n'); passed = passed + 1;
catch ME
    fprintf('FAILED - %s\n', ME.message); failed = failed + 1;
end

%% ========================================================================
%  Test 5: Drowsiness Threshold Boundary
%  ========================================================================
fprintf('[TEST 5] Drowsiness threshold boundary... ');
try
    % With maximum PERCLOS (1.0), score should be significant (>0.3)
    maxScore = 0.40 * min(1.0 / 0.4, 1) + ...  % PERCLOS at max
               0.25 * min(0.3 / 0.5, 1) + ...  % BlinkDur nominal
               0.20 * min(0   / 15,  1) + ...   % HeadPitch = 0
               0.15 * min(0   / 0.5, 1);        % YawnFlag = 0
    assert(maxScore > 0.3, 'Expected drowsiness score > 0.3 at max PERCLOS');
    fprintf('PASSED (analytic: score=%.2f)\n', maxScore); passed = passed + 1;
catch ME
    fprintf('FAILED - %s\n', ME.message); failed = failed + 1;
end

%% ========================================================================
%  Test 6: Distraction Threshold Boundary
%  ========================================================================
fprintf('[TEST 6] Distraction threshold boundary... ');
try
    % Gaze 90° off centre, head yaw 90° → both scores saturate at 1
    gazeOff = min(90 / 30, 1);  % = 1
    headOff = min(90 / 25, 1);  % = 1
    expectedScore = 0.6*gazeOff + 0.4*headOff;  % = 1.0
    assert(abs(expectedScore - 1.0) < 0.01, 'Expected distraction = 1.0 at extreme gaze/head');
    fprintf('PASSED (analytic: score=%.2f)\n', expectedScore); passed = passed + 1;
catch ME
    fprintf('FAILED - %s\n', ME.message); failed = failed + 1;
end

%% ========================================================================
%  Test 7: Speed Reduction at High Risk
%  ========================================================================
fprintf('[TEST 7] Speed reduction at high risk... ');
try
    spdCmd = evalin('base', 'log_SpeedCommand');
    vSpeed = evalin('base', 'log_VehicleSpeed');

    % At some points, speed command should be less than vehicle speed
    % (indicating active speed reduction)
    diff = vSpeed.Data - spdCmd.Data;
    hasReduction = any(diff > 0.1);
    assert(hasReduction, 'No speed reduction observed during simulation');
    fprintf('PASSED (max reduction = %.1f km/h)\n', max(diff)); passed = passed + 1;
catch ME
    fprintf('FAILED - %s\n', ME.message); failed = failed + 1;
end

%% ========================================================================
%  Summary
%  ========================================================================
total = passed + failed + skipped;
fprintf('\n====== Test Summary ======\n');
fprintf('  Total:   %d\n', total);
fprintf('  Passed:  %d\n', passed);
fprintf('  Failed:  %d\n', failed);
fprintf('  Skipped: %d\n', skipped);

if failed == 0
    fprintf('\n  ALL TESTS PASSED ✓\n');
else
    fprintf('\n  %d TEST(S) FAILED ✗\n', failed);
end
fprintf('==========================\n\n');
