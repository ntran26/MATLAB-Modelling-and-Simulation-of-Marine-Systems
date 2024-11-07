mdl = "AutopilotDDPG";
open_system(mdl);

% Define observation and action specifications based on your model setup
obsInfo = rlNumericSpec([2, 1], 'LowerLimit', -inf, 'UpperLimit', inf); 
actInfo = rlNumericSpec([1, 1], 'LowerLimit', -10*pi/180, 'UpperLimit', 10*pi/180); 

% Create the environment object
env = rlSimulinkEnv(mdl, 'AutopilotDDPG/DRLagent', obsInfo, actInfo);