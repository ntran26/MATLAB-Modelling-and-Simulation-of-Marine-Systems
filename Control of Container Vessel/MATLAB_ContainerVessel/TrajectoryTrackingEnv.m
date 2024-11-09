classdef TrajectoryTrackingEnv < rl.env.MATLABEnvironment
    properties
        % State variables (relevant states from container model)
        State = [2; 0; 0; 0; 0; 0; 0; 0; 0; 80];  % Initial state: [u; v; r; x; y; psi; p; phi; delta; n]
        
        % Simulation parameters
        SampleTime = 0.1;        % Simulation time step
        DesiredSpeed = 5;        % Desired speed [m/s]
        
        % Waypoints for trajectory tracking
        WaypointsX = [0, 1000, 1000, 2000, 4000];
        WaypointsY = [0, 1000, 3000, 5000, 5000];
        AcceptanceRadius = 175/2;  % Radius of acceptance for each waypoint
        CurrentWaypointIndex = 1;  % Index of the current target waypoint
    end
    
    properties(Access = protected)
        % Internal property to store the done flag
        IsDone = false;
    end
    
    methods
        function this = TrajectoryTrackingEnv()
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
            % Extract actions
            delta_c = action(1);  % Rudder angle command
            n_c = action(2);      % Shaft speed command
            
            % Run ship dynamics
            [Xdot, U] = container(this.State, [delta_c; n_c]);
            this.State = this.State + this.SampleTime * Xdot;
            
            % Enforce yaw angle within [-pi, pi] range
            this.State(6) = wrapToPi(this.State(6));
            
            % Get current waypoint position
            targetX = this.WaypointsX(this.CurrentWaypointIndex);
            targetY = this.WaypointsY(this.CurrentWaypointIndex);
            
            % Calculate distance to the current waypoint
            dx = targetX - this.State(4);  % Difference in x-position
            dy = targetY - this.State(5);  % Difference in y-position
            dist_to_wpt = sqrt(dx^2 + dy^2);
            
            % Check if the waypoint is reached
            if dist_to_wpt <= this.AcceptanceRadius && this.CurrentWaypointIndex < length(this.WaypointsX)
                this.CurrentWaypointIndex = this.CurrentWaypointIndex + 1;
                % Update to the new target waypoint
                targetX = this.WaypointsX(this.CurrentWaypointIndex);
                targetY = this.WaypointsY(this.CurrentWaypointIndex);
                dx = targetX - this.State(4);
                dy = targetY - this.State(5);
                dist_to_wpt = sqrt(dx^2 + dy^2);
            end
            
            % Desired heading angle toward the waypoint
            desiredHeading = atan2(dy, dx);
            
            % Calculate heading and speed errors
            headingError = wrapToPi(desiredHeading - this.State(6));
            speedError = this.DesiredSpeed - this.State(1);
            
            % Reward: negative of squared errors in heading, speed, and distance to waypoint
            reward = - sqrt(headingError^2 + speedError^2 + 0.1 * dist_to_wpt^2);
            
            % Check if all waypoints are reached
            isDone = this.CurrentWaypointIndex == length(this.WaypointsX) && dist_to_wpt <= this.AcceptanceRadius;
            this.IsDone = isDone;
            
            % Return next observation, reward, done flag, and logged signals
            nextObs = this.State;
            loggedSignals = [];
        end
        
        function initialObservation = reset(this)
            % Reset state and return initial observation
            u0 = 2;            % Initial surge velocity
            n_c = 80;          % Initial shaft speed
            this.State = [u0; 0; 0; 0; 0; 0; 0; 0; 0; n_c];  % Reset to initial state
            this.CurrentWaypointIndex = 1;  % Reset waypoint index
            this.IsDone = false;
            initialObservation = this.State;
        end
    end
end
