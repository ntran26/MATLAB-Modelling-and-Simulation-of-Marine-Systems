classdef SpeedControlEnv < rl.env.MATLABEnvironment
    properties
        % State variables (relevant states from container model)
        State = [8; 0; 0; 0; 0; 0; 0; 0; 0; 80];  % Initial state: [u; v; r; x; y; psi; p; phi; delta; n]
        
        % Simulation parameters
        SampleTime = 0.1;                % Simulation time step
        DesiredHeading = 20 * pi / 180;  % Desired course (radians)
        DesiredSpeed = 10;               % Desired speed (m/s)
    end
    
    properties(Access = protected)
        % Internal property to store the done flag
        IsDone = false;
    end
    
    methods
        function this = SpeedControlEnv()
            % Define observation space (10-dimensional state vector)
            ObservationInfo = rlNumericSpec([10 1], ...
                'LowerLimit', -inf, 'UpperLimit', inf);
            ObservationInfo.Name = 'Ship States';
            ObservationInfo.Description = '[u; v; r; x; y; psi; p; phi; delta; n]';

            % Define action space (continuous [rudder angle, shaft speed])
            ActionInfo = rlNumericSpec([2 1], ...
                'LowerLimit', [-10*pi/180; 1], ...
                'UpperLimit', [10*pi/180; 200]);
            ActionInfo.Name = 'Rudder Angle and Shaft Speed';

            % Initialize the RL environment with observation and action info
            this = this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
            
            % Set initial state
            reset(this);
        end
        
        function [nextObs, reward, isDone, loggedSignals] = step(this, action)
            % Apply action and update the state based on ship dynamics
            
            % Extract actions
            delta_c = action(1);  % Rudder angle command
            n_c = action(2);      % Shaft speed command
            
            % Run ship dynamics
            [Xdot, U] = container(this.State, [delta_c; n_c]);
            this.State = this.State + this.SampleTime * Xdot;
            
            % Enforce yaw angle within [-pi, pi] range
            this.State(6) = wrapToPi(this.State(6));
            
            % Compute heading and speed errors
            headingError = wrapToPi(this.DesiredHeading - this.State(6));
            speedError = this.DesiredSpeed - this.State(1);

            % Reward: negative of squared errors in heading and speed
            reward = - sqrt(headingError^2 + speedError^2);
            
            % Check terminal condition
            isDone = abs(headingError) < 0.05 && abs(speedError) < 0.1;
            this.IsDone = isDone;
            
            % Return next observation, reward, done flag, and logged signals
            nextObs = this.State;
            loggedSignals = [];
        end
        
        function initialObservation = reset(this)
            % Reset state and return initial observation
            u0 = 8;            % Initial surge velocity
            n_c = 80;          % Initial shaft speed
            this.State = [u0; 0; 0; 0; 0; 0; 0; 0; 0; n_c];  % Reset to initial state
            this.IsDone = false;
            initialObservation = this.State;
        end
    end
end