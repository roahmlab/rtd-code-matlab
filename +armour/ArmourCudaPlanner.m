classdef ArmourCudaPlanner < rtd.planner.RtdPlanner & rtd.util.mixins.Options & rtd.util.mixins.UUID & rtd.util.mixins.NamedClass
    properties
        server_address
        server_port

        connected_socket

        errored = false

        trajectory_factory
    end
    
    methods (Static)
        function options = defaultoptions()
            options.server_address = '127.0.0.1';
            options.server_port = 65535;
            options.connect_timeout = 5;
            options.packet_timeout = 0.1;
            options.verboseLevel = 'DEBUG';
        end
    end

    % This class specifies the overall planner, so it sets up everything
    % for any special type of rtd.planner.RtdPlanner.
    methods
        % The constructor would do the following
        % - determine RTD_TrajOpt parameters
        % - Construct OptimizationEngine(s) with parameters
        % - Construct ReachableSets for each reachable set and trajectory type
        % - Construct some Objective object for problem at hand
        
        % - Create RTD_TrajOpt for each trajectory type
        % - Create initial trajectories for each trajectory type
        function self = ArmourCudaPlanner(trajOptProps, robot, options)
            arguments
                trajOptProps (1,1) rtd.planner.trajopt.TrajOptProps
                robot (1,1) armour.ArmourAgent
                options.server_address (1,:) {mustBeTextScalar}
                options.server_port (1,1) uint32
                options.connect_timeout (1,1) double
                options.packet_timeout (1,1) double
                options.verboseLevel (1,1) rtd.util.types.LogLevel
            end
            
            % Get complete options
            options = self.mergeoptions(options);
            
            % Ouput logging
            self.set_vdisplevel(options.verboseLevel);

            % Attempt to connect to the server
            self.vdisp('Attempting to connect to the CUDA planner server.')
            self.connected_socket = tcpclient(options.server_address, options.server_port, ...
                "ConnectTimeout", options.connect_timeout, ...
                "Timeout", options.packet_timeout);

            % Configure our connection
            if ~self.initializeConnection(trajOptProps, robot)
                self.closeConnection()
                error("FAILED")
            end

            self.trajectory_factory = armour.trajectory.ArmTrajectoryFactory(trajOptProps, 'bernstein');
        end

        % Then on each waypoint, we call for a trajectory plan:
        % Use RTD to solve for a trajectory and return either
        % the parameters or invalid signal (continue)
        function [trajectory, info] = planTrajectory(self, robotState, worldState, waypoint)
            arguments
                self armour.ArmourCudaPlanner
                robotState rtd.entity.states.ArmRobotState
                worldState
                waypoint
            end
            inputs.q_0 = robotState.position;
            inputs.q_dot_0 = robotState.velocity;
            inputs.q_ddot_0 = robotState.acceleration;
            inputs.q_des = waypoint;
            inputs.num_obs = length(worldState.obstacles);
            inputs.obs = struct;
            for i=1:inputs.num_obs
                temp = reshape(worldState.obstacles(i).Z, [1,size(worldState.obstacles(i).Z,1) * size(worldState.obstacles(i).Z,2)]);
                inputs.obs.(['obs' num2str(i)]) = temp;
            end
            plan_id = self.startPlan(inputs);
            [success, info] = self.requestPlan(plan_id);
            
            trajectory = [];
            if success
                while isempty(fields(info))
                    [success, info] = self.requestPlan(plan_id);
                    if ~success
                        error("failed!")
                    end
                end
                if ~isempty(info.parameters)
                    jrs = armour.reachsets.JRSInstance;
                    jrs.n_t = 128;
                    jrs.n_q = 7;
                    jrs.n_k = 7;
                    jrs.num_parameters = jrs.n_k;
                    c_k = zeros(7,1);
                    % !!!!!!
                    % Make sure this is consistent with the k_range in
                    % kinova_src/kinova_simulator_interfaces/kinova_planner_realtime/Parameters.h 
                    % !!!!!!
                    g_k = [pi/24; pi/24; pi/24; pi/24; pi/24; pi/24; pi/30];
                    jrs.input_range = ones(jrs.n_k, 1) * jrs.input_range;
                    jrs.output_range = c_k + [-1.0, 1.0] .* g_k;
    
                    trajectory = self.trajectory_factory.createTrajectory(robotState, jrsInstance=jrs);
                    trajectory.setParameters(info.parameters);
                end
            end
        end

        function delete(self)
            if ~isempty(self.connected_socket)
                self.closeConnection();
            end
        end
    end

    methods (Access=private)
        function success = initializeConnection(self, trajOptProps, robot)
            % Send robot_setup packet
            robot_setup.planner_id = self.uuid;
            success = self.sendRequest('robot_setup', robot_setup);
        end

        function closeConnection(self)
            % Close connection and set current planner to invalid
            cleanup.planner_id = self.uuid;
            self.sendRequest('clear_planner', cleanup);
            self.connected_socket = [];
        end

        function plan_uuid = startPlan(self, inputs, options)
            arguments
                self armour.ArmourCudaPlanner
                inputs struct
                options.keepalive_timeout (1,1) double = 1.0
                options.request_constraints (1,1) logical = false
                options.request_joint_frs (1,1) logical = false
                options.request_input_frs (1,1) logical = false
                options.reinitialize_if_disconnected (1,1) logical = false
            end
            % Setup all the information for planning
            plan_info.id = rtd.functional.uuid;
            plan_info.planner_id = self.uuid;
            plan_info.keepalive_timeout = options.keepalive_timeout;
            plan_info.outputs = ["result", "time"];
            if options.request_constraints
                plan_info.outputs = [plan_info.outputs, "constraints"];
            end
            if options.request_joint_frs
                plan_info.outputs = [plan_info.outputs, "request_joint_frs"];
            end
            if options.request_input_frs
                plan_info.outputs = [plan_info.outputs, "request_input_frs"];
            end
            
            % Add the input data
            plan_info.inputs = inputs;

            if self.sendRequest('plan_trajectory', plan_info)
                plan_uuid = plan_info.id;
            elseif self.errored && options.reinitialize_if_disconnected
                self.vdisp("Attempting reconnect", "WARN");
                error("I LIED")
            else
                plan_uuid = [];
            end
        end

        function [success, result] = requestPlan(self, plan_uuid, options)
            arguments
                self armour.ArmourCudaPlanner
                plan_uuid (1,:) char
                options.clear (1,1) logical = true
                options.request_type = 'ALL'
                options.block (1,1) logical = true
                options.block_timeout (1,1) double = 5
            end
            request.plan_id = plan_uuid;
            request.planner_id = self.uuid;
            request.type = options.request_type;
            request.clear = options.clear;

            result = [];
            success = false;
            if self.sendRequest('request_result', request)
                success = true;
                if options.block
                    start_time = tic;
                    duration = toc(start_time);
                    while self.connected_socket.NumBytesAvailable == 0 && duration < options.block_timeout
                        pause(0.01);
                        duration = toc(start_time);
                    end
                end
                res = self.connected_socket.read();
                if ~isempty(res)
                    try
                        result = jsondecode(char(res));
                    catch
                        char(res);
                    end
                end
            end     
        end

        function success = stopPlan(self, plan_uuid)
            arguments
                self armour.ArmourCudaPlanner
                plan_uuid (1,:) char
            end
            
        end

        function success = sendRequest(self, message_type, struct_data, options)
            arguments
                self armour.ArmourCudaPlanner
                message_type (1,:) {mustBeTextScalar}
                struct_data
                options.retry_count (1,1) uint8 = 3
                options.add_checksum (1,1) logical = true
            end
            
            % Get the initial message
            packet.request = char(message_type);
            packet.data = struct_data;
            message = jsonencode(packet);

            % Checksum
            checksum = char(mod(sum(message), 256));

            % encode a uint32 length for the number of bytes to read.
            message_length = length(message);
            if message_length > 256^4
                error("TOO LONG")
            end
            % big byteorder
            length_string = char(flip(typecast(uint32(message_length), 'uint8')));

            % Combine message
            message = [length_string, message, checksum];

            success = false;
            % Send as many times as needed
            for i=1:options.retry_count
                try
                    self.connected_socket.write(message);
                    resp = self.connected_socket.read(3);
                    if strcmp(char(resp),'OK!')
                        success = true;
                        break;
                    elseif strcmp(char(resp),'BAD')
                        continue;
                    else
                        error("BAD RESPONSE FROM SERVICE")
                    end
                catch ME
                    % MATLAB:networklib:tcpclient:writeFailed
                    % MATLAB:networklib:tcpclient:readFailed
                    % MATLAB:networklib:tcpclient:connectTerminated
                    disp(ME)
                end
            end
        end

        function errorfcn(self)
            self.errored = true;
        end
    end
end