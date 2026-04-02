%% DMS_CONFIG - Configuration parameters for the Driver Monitoring System
%
%  This script defines all tunable parameters, thresholds, and constants
%  used by the DMS ADAS Simulink model. Run this before dms_adas_project.m
%  to load parameters into the base workspace.
%
%  Copyright (c) 2026 DMS ADAS Project

%% ========================================================================
%  Simulation Settings
%  ========================================================================
Config.Simulation.StopTime       = 300;    % seconds
Config.Simulation.SampleTime     = 0.02;   % 50 Hz base rate (seconds)
Config.Simulation.SolverType     = 'FixedStepDiscrete';
Config.Simulation.FixedStep      = '0.02';

%% ========================================================================
%  Sensor Parameters
%  ========================================================================

% --- Camera / Face Detection ---
Config.Sensor.Camera.FrameRate       = 30;     % frames per second
Config.Sensor.Camera.Resolution      = [640, 480]; % [width, height] pixels
Config.Sensor.Camera.NoiseStdDev     = 0.02;   % normalised noise std-dev

% --- Eye Tracking ---
Config.Sensor.Eye.PERCLOS_WindowSec  = 60;     % PERCLOS rolling window (s)
Config.Sensor.Eye.BlinkRateNormal    = 15;     % blinks per minute (nominal)
Config.Sensor.Eye.GazeSampleRate     = 50;     % Hz

% --- Head Pose ---
Config.Sensor.HeadPose.YawRange      = [-90, 90];   % degrees
Config.Sensor.HeadPose.PitchRange    = [-45, 45];    % degrees
Config.Sensor.HeadPose.NoiseStdDev   = 1.0;          % degrees

%% ========================================================================
%  Drowsiness Detection Thresholds
%  ========================================================================
Config.Drowsiness.PERCLOS_Threshold      = 0.4;   % fraction eyes-closed
Config.Drowsiness.BlinkDuration_Thresh   = 0.5;   % seconds
Config.Drowsiness.HeadNodAngle_Thresh    = 15;    % degrees (pitch)
Config.Drowsiness.YawnDuration_Thresh    = 2.0;   % seconds
Config.Drowsiness.FusionWeights          = [0.40, 0.25, 0.20, 0.15];
%                                           PERCLOS, BlinkDur, HeadNod, Yawn

%% ========================================================================
%  Distraction Detection Thresholds
%  ========================================================================
Config.Distraction.GazeOff_AngleThresh   = 30;    % degrees from road centre
Config.Distraction.GazeOff_TimeThresh    = 2.0;   % seconds continuously off
Config.Distraction.HeadTurn_Thresh       = 25;    % degrees yaw
Config.Distraction.PhoneUse_Confidence   = 0.6;   % classifier confidence

%% ========================================================================
%  Fatigue Estimation Parameters
%  ========================================================================
Config.Fatigue.SteeringEntropy_Thresh    = 0.7;   % normalised entropy
Config.Fatigue.LaneDev_StdThresh         = 0.3;   % metres
Config.Fatigue.DrivingDuration_Warn      = 7200;  % seconds (2 hours)
Config.Fatigue.DrivingDuration_Critical  = 14400; % seconds (4 hours)
Config.Fatigue.FusionWeights             = [0.35, 0.30, 0.20, 0.15];
%                                           SteerEntropy, LaneDev, Duration, EyeMetrics

%% ========================================================================
%  Alert Levels and Timing
%  ========================================================================
%  Level 0 = None, 1 = Low, 2 = Medium, 3 = High, 4 = Critical
Config.Alert.NumLevels           = 5;  % 0..4
Config.Alert.Thresholds          = [0.0, 0.3, 0.5, 0.7, 0.9];
Config.Alert.EscalationDelay     = [0, 5, 3, 2, 0]; % seconds per level
Config.Alert.CooldownTime        = 10;  % seconds before de-escalation

% --- Modality Enables ---
Config.Alert.Visual.Enable       = true;
Config.Alert.Audio.Enable        = true;
Config.Alert.Haptic.Enable       = true;

% --- Audio ---
Config.Alert.Audio.ToneFreq      = [0, 440, 880, 1760, 2200]; % Hz per level
Config.Alert.Audio.ToneDuration  = [0, 0.5, 0.5, 1.0, 2.0];  % seconds

%% ========================================================================
%  Vehicle Intervention Parameters
%  ========================================================================
Config.Intervention.LaneKeepAssist.Enable   = true;
Config.Intervention.LaneKeepAssist.Gain     = 0.5;
Config.Intervention.SpeedReduction.Enable   = true;
Config.Intervention.SpeedReduction.MaxDecel = 2.0;  % m/s^2
Config.Intervention.EmergencyStop.Enable    = true;
Config.Intervention.EmergencyStop.Decel     = 5.0;  % m/s^2

%% ========================================================================
%  Data Logging
%  ========================================================================
Config.Logging.Enable            = true;
Config.Logging.Decimation        = 5;    % log every Nth sample
Config.Logging.MaxPoints         = 15000;
Config.Logging.Signals           = { ...
    'DrowsinessScore', 'DistractionScore', 'FatigueScore', ...
    'AlertLevel', 'PERCLOS', 'GazeAngle', 'HeadYaw', 'HeadPitch', ...
    'VehicleSpeed', 'SteeringTorque', 'LateralDeviation'};

%% ========================================================================
%  Publish to Base Workspace
%  ========================================================================
% Flatten key scalars into base workspace variables for Simulink constant
% blocks (Simulink resolves variable names from the base workspace).
assignin('base', 'Ts',                       Config.Simulation.SampleTime);
assignin('base', 'PERCLOS_Threshold',        Config.Drowsiness.PERCLOS_Threshold);
assignin('base', 'BlinkDuration_Thresh',     Config.Drowsiness.BlinkDuration_Thresh);
assignin('base', 'HeadNodAngle_Thresh',      Config.Drowsiness.HeadNodAngle_Thresh);
assignin('base', 'YawnDuration_Thresh',      Config.Drowsiness.YawnDuration_Thresh);
assignin('base', 'Drowsiness_Weights',       Config.Drowsiness.FusionWeights);
assignin('base', 'GazeOff_AngleThresh',      Config.Distraction.GazeOff_AngleThresh);
assignin('base', 'GazeOff_TimeThresh',       Config.Distraction.GazeOff_TimeThresh);
assignin('base', 'HeadTurn_Thresh',          Config.Distraction.HeadTurn_Thresh);
assignin('base', 'PhoneUse_Confidence',      Config.Distraction.PhoneUse_Confidence);
assignin('base', 'Fatigue_Weights',          Config.Fatigue.FusionWeights);
assignin('base', 'SteeringEntropy_Thresh',   Config.Fatigue.SteeringEntropy_Thresh);
assignin('base', 'LaneDev_StdThresh',        Config.Fatigue.LaneDev_StdThresh);
assignin('base', 'DriveDuration_Warn',       Config.Fatigue.DrivingDuration_Warn);
assignin('base', 'DriveDuration_Critical',   Config.Fatigue.DrivingDuration_Critical);
assignin('base', 'Alert_Thresholds',         Config.Alert.Thresholds);
assignin('base', 'CooldownTime',             Config.Alert.CooldownTime);
assignin('base', 'LKA_Gain',                 Config.Intervention.LaneKeepAssist.Gain);
assignin('base', 'MaxDecel',                 Config.Intervention.SpeedReduction.MaxDecel);
assignin('base', 'EmergencyDecel',           Config.Intervention.EmergencyStop.Decel);
assignin('base', 'CameraNoise',              Config.Sensor.Camera.NoiseStdDev);
assignin('base', 'HeadPoseNoise',            Config.Sensor.HeadPose.NoiseStdDev);
assignin('base', 'Config',                   Config);

fprintf('DMS Configuration loaded into base workspace.\n');
