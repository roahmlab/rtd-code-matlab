classdef ArmourCudaPlanner < rtd.planner.RtdPlanner & rtd.util.mixins.Options & rtd.util.mixins.UUID & rtd.util.mixins.NamedClass
    properties
        server_address
        server_port

        trajOptProps
        robot

        connected_socket
        socket_fn
        connection_lost = false

        trajectory_factory
        endian_transformer

        read_timeout = false
    end

    properties (Dependent)
        read_available_or_timeout (1,1) logical
    end

    methods
        function setTimeoutState(self, state)
            self.read_timeout = state;
        end
        function tf = get.read_available_or_timeout(self)
            tf = self.read_timeout ...
                || (~isempty(self.connected_socket) ...
                    && self.connected_socket.NumBytesAvailable > 0);
        end
    end
    
    methods (Static)
        function options = defaultoptions()
            options.server_address = '127.0.0.1';
            options.server_port = 65535;
            options.connect_timeout = 5;
            options.packet_timeout = 0.5;
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
            
            self.trajOptProps = trajOptProps;
            self.robot = robot;

            % Endian transformer is a NOP to start
            self.endian_transformer = @(x)x;
            test_endian = typecast(uint16(1),'uint8');
            % If we are little endian, make endian_transformer swap
            if test_endian(1)
                self.endian_transformer = @(x)swapbytes(x);
            end

            % Get complete options
            options = self.mergeoptions(options);
            
            % Ouput logging
            self.set_vdisplevel(options.verboseLevel);

            % Attempt to connect to the server
            self.vdisp('Attempting to connect to the CUDA planner server.')
            % Configure our connection
            self.socket_fn = @() tcpclient(options.server_address, options.server_port, ...
                "ConnectTimeout", options.connect_timeout, ...
                "Timeout", options.packet_timeout, ...
                "EnableTransferDelay", false);
            if ~self.initializeConnection(trajOptProps, robot)
                self.closeConnection()
                error("FAILED TO CONNNECT")
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
            start_time = tic;
            plan_id = self.startPlan(inputs);
            [success, info] = self.requestPlan(plan_id);
            
            trajectory = [];
            if success
                while ~strcmp(info.result, 'plan_complete')
                    [success, info] = self.requestPlan(plan_id);
                    if ~success
                        error("Plan request failed! Check connection quality or timeout!")
                    end
                    if strcmp(info.result, 'plan_failed')
                        error("Remote planning call failed!")
                    end
                    if strcmp(info.result, 'request_error')
                        warning("Encountered bad data in buffer, retrying request!")
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
                self.vdisp("Remote planner time: " + string(info.time/1000),"INFO");
                self.vdisp("Full planning call time: " + string(toc(start_time)),"INFO");
            end
        end

        function delete(self)g
            if ~isempty(self.connected_socket)
                self.closeConnection();
            end
        end
    end

    methods (Access=private)
        function success = initializeConnection(self, trajOptProps, robot)
            % Send robot_setup packet
            robot_setup.planner_id = self.uuid;

            % Setup and send
            self.connected_socket = self.socket_fn();
            success = self.sendRequest('robot_setup', robot_setup);
            self.connection_lost = ~success;
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
                options.reinitialize_if_disconnected (1,1) logical = true
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
            elseif self.connection_lost && options.reinitialize_if_disconnected
                self.vdisp("Attempting reconnect", "WARN");
%                 error("I LIED")
                if self.initializeConnection(self.trajOptProps, self.robot)
                    extra_options = namedargs2cell(options);
                    plan_uuid = self.startPlan(inputs, extra_options{:});
                else
                    error("Reconnect failed!");
                end
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
                options.request_timeout = 0.4
                options.block (1,1) logical = true
                options.block_timeout (1,1) double = 1
            end
            request.plan_id = plan_uuid;
            request.planner_id = self.uuid;
            request.type = options.request_type;
            request.timeout = options.request_timeout;
            request.clear = options.clear;

            result = struct;
            success = false;
            if self.sendRequest('request_result', request)
                success = true;
                result.result = 'request_error';
                res = self.readSocket(block=options.block, block_timeout=options.block_timeout);
                if ~isempty(res)
                    try
                        result = self.processMessage(res);
                    catch
%                         result.result = 'request_error';
                    end
                end
            end
        end

        function msg = processMessage(self, message)
            msg = struct;
            if length(message) < 6
                error("BAD MESSAGE")
                return
            end
            
            msg_len_chk = computeMsgLenChk(message(1:3));

            if any(msg_len_chk ~= message(4:5))
                error("BAD MESSAGE")
                return
            end

            msg_len = [0, message(1:3)];
            msg_len = typecast(uint8(msg_len), 'uint32');
            msg_len = self.endian_transformer(msg_len);

            try
                msg_prop = message(6:5+msg_len);
                msg_chk = message(6+msg_len);
            catch
                error("BAD MESSAGE")
                return
            end
            
            if computeMsgChk(msg_prop) ~= msg_chk
                error("BAD MESSAGE")
                return
            end

            msg = jsondecode(char(msg_prop));
        end

        function read_out = readSocket(self, options)
            arguments
                self armour.ArmourCudaPlanner
                options.block = false
                options.block_timeout = self.instanceOptions.packet_timeout
            end
            if options.block
                if ~isempty(options.block_timeout)
                    self.read_timeout = false;
                    read_timer = timer('TimerFcn', @(h,e)self.setTimeoutState(true), 'StartDelay', options.block_timeout);
                    start(read_timer)
                end
                waitfor(self,'read_available_or_timeout',true);
                try stop(read_timer); catch; end
                % The following is needed to ensure the class destructor
                % actually runs
                try delete(read_timer); catch; end
            end
            read_out = self.connected_socket.read();
        end

        function success = stopPlan(self, plan_uuid)
            arguments
                self armour.ArmourCudaPlanner
                plan_uuid (1,:) char
            end
            % Setup all the information for planning
            plan_info.id = plan_uuid;
            plan_info.planner_id = self.uuid;

            success = self.sendRequest('stop_plan', plan_info);
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
            checksum = computeMsgChk(message);

            % encode a uint32 length for the number of bytes to read.
            % we're only gonna save uint24 though (matlab has no type for
            % that)
            message_length = uint32(length(message));
            if message_length > 256^3
                error("TOO LONG")
            end
            % big endian byteorder
            length_string = typecast(self.endian_transformer(message_length), 'uint8');
            length_string = length_string(2:end);
            length_chksum = computeMsgLenChk(length_string);

            length_string = char([length_string, length_chksum]);

            % Combine message
            message = [length_string, message, checksum];

            success = false;
            % Send as many times as needed
            for i=1:options.retry_count
                try
                    self.connected_socket.flush();
                    self.connected_socket.write(message);
                    resp = self.connected_socket.read(3);
%                     resp = self.readSocket(3, block=true);
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
                    % MATLAB:class:InvalidHandle
                    if strcmp(ME.identifier,'MATLAB:networklib:tcpclient:readFailed')
                        % WARNING READ TIMED OUT, FLUSHING AND RESENDING
                        self.connected_socket.flush();
                    elseif strcmp(ME.identifier,'MATLAB:networklib:tcpclient:connectTerminated') ...
                            || strcmp(ME.identifier,'MATLAB:class:InvalidHandle')
                        warning("Connection to server lost!")
                        self.connection_lost = true;
                        self.connected_socket = [];
                        break;
                    elseif strcmp(ME.identifier, 'MATLAB:structRefFromNonStruct')
                        warning("Invalid socket!")
                    else
                        disp(ME)
                    end
                end
            end
        end

%         function message = readResponse(self)


        function errorfcn(self)
            self.connection_lost = true;
        end
    end
end

function chksum = computeMsgChk(message)
    chksum = char(mod(sum(message), 256));
end

function chksum = computeMsgLenChk(msg_len_str)
    length_sum_chksum = uint8(mod(sum(msg_len_str), 256));
    length_xor_chksum = uint8(255);
    for elem=msg_len_str
        length_xor_chksum = bitxor(length_xor_chksum, elem);
    end
    chksum = char([length_sum_chksum, length_xor_chksum]);
end
