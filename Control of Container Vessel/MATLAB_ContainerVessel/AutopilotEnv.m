classdef AutopilotEnv < rl.env.MATLABEnvironment
    properties
        % State variables (relevant states from container model)
        % initial [u; v; r; x; y; psi; p; phi; delta; n]
        State = [8; 0; 0; 0; 0; 0; 0; 0; 0; 70];  
        
        % Simulation parameters
        SampleTime = 0.1;           % Simulation time step
        DesiredHeading = 10*pi/180; % Desired course (rad)
    end
    
    properties(Access = protected)
        % Last action taken (rudder angle)
        LastAction = 0;
    end
    
    methods
        function this = AutopilotEnv()
            % Define observation (yaw) and action (rudder) specifications
            ObservationInfo = rlNumericSpec([1 1], 'LowerLimit', -pi, 'UpperLimit', pi);
            ActionInfo = rlNumericSpec([1 1], 'LowerLimit', -10*pi/180, 'UpperLimit', 10*pi/180);
            this = this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
        end
        
        function [nextObs, reward, isDone, loggedSignals] = step(this, action)
            % Dynamics and reward calculation
            rudderAngle = action;
            
            % Define control input vector [delta_c; n_c]
            % Assuming constant commanded shaft RPM for simplicity
            n_c = 70; 
            controlInput = [rudderAngle; n_c];
            
            % Call the container function to get state derivatives
            [Xdot, ~] = container(this.State, controlInput);
            
            % Update state using Euler integration
            this.State = this.State + Xdot * this.SampleTime;
            
            % Calculate the yaw angle and wrap within -pi to pi
            yawAngle = wrapToPi(this.State(6));
            this.State(6) = yawAngle;  % Update yaw angle in state vector
            
            % Calculate reward based on heading error
            headingError = wrapToPi(this.DesiredHeading - yawAngle);
            reward = -abs(headingError);  % Reward is negative of heading error magnitude
            
            % Update observation (yaw angle)
            nextObs = yawAngle;
            loggedSignals = struct('HeadingError', headingError);
            
            % Terminal condition (optional)
            isDone = false;
        end
        
        function initialState = reset(this)
            % Reset environment to initial state
            this.State = [8; 0; 0; 0; 0; 0; 0; 0; 0; 70];  % Reset to initial state
            initialState = this.State(6);  % Return initial yaw angle
        end
    end
end