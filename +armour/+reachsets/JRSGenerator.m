classdef JRSGenerator < rtd.planner.reachsets.ReachSetGenerator
    % JointReachableSetsOnline
    % This does the online computation of joint reachable sets. It then
    % generates a JRSInstance object.

    % Required Abstract Properties
    properties
        cache_max_size = 1
    end

    % Additional Properties
    properties
        taylor_degree
        add_ultimate_bound
        traj_type
        joint_axes
        controller
    end

    properties
        num_q
        param_center
        param_range
        tplan
        tfinal
        tdiscretization
        num_t
        id_map
        ultimate_bound
        k_r
        base_params
        k_ids
        times
        error_pos
        error_vel
    end

    % Properties for the generator
    methods
        function self = JRSGenerator(robot, options)
            arguments
                robot armour.ArmourAgent
                options.taylor_degree(1,1) double {mustBeInteger} = 1
                options.add_ultimate_bound(1,1) logical = true
                options.traj_type {mustBeMember(options.traj_type,{'piecewise','bernstein'})} = 'piecewise'
                options.verboseLevel(1,1) rtd.util.types.LogLevel = "DEBUG"
                options.param_center(:,1) double = 0
                options.param_range(:,1) double = pi/36
                options.tplan(1,1) double = 0.5
                options.tfinal(1,1) double = 1
                options.unique_tid(1,1) logical = false
                options.tdiscretization(1,1) double = 0.01
            end
            
            self.joint_axes = [robot.info.joints.axes];
            self.controller = robot.controller;
            self.taylor_degree = options.taylor_degree;
            self.add_ultimate_bound = options.add_ultimate_bound;
            self.traj_type = options.traj_type;
            self.set_vdisplevel(options.verboseLevel);


            % START REWRITE
            self.num_q = robot.info.num_q;
            self.param_center = ones(self.num_q, 1).*options.param_center;
            self.param_range = [self.param_center - options.param_range, self.param_center + options.param_range];

            self.tplan = options.tplan;
            self.tfinal = options.tfinal;
            self.tdiscretization = options.tdiscretization;
            num_t = ceil(options.tfinal/options.tdiscretization);
            self.num_t = num_t;

            self.id_map = struct;
            try
                self.ultimate_bound = robot.controller.ultimate_bound;
                self.k_r = robot.controller.k_r;
            catch
                self.ultimate_bound = [];
                self.k_r = [];
            end

            % Create base PZ's for parameters
            % Note that cora doesn't display arrays of PZ's correctly
            % so this will always error out if printed in the command window.
            % You will have to index each one individually.
            base_params(self.num_q) = armour.pz_roahm.polyZonotope_ROAHM();
            self.k_ids = 1:self.num_q;
            for i = 1:self.num_q
                % Make the base pz
                base_params(i) = armour.pz_roahm.polyZonotope_ROAHM(0, 1, [], 1, self.k_ids(i));
                % Add its id to the map
                self.id_map.(sprintf('k%d', i)) = self.k_ids(i);
            end
            % Convert to cell so we it's not a pain to debug
            self.base_params = num2cell(base_params);

            % Create the base PZ's for time
            if options.unique_tid
                t_id = length(fieldnames(self.id_map)) + 1:num_t;
            else
                t_id = length(fieldnames(self.id_map)) + 1;
                self.id_map.(sprintf('t%d', t_id)) = t_id;
            end
            times(num_t) = armour.pz_roahm.polyZonotope_ROAHM();
            for i = 1:num_t
                % Make the base pz
                times(i) = armour.pz_roahm.polyZonotope_ROAHM(self.tdiscretization*(i-1)+self.tdiscretization/2, self.tdiscretization/2, [], 1, t_id);
                if options.unique_tid
                    self.id_map.(sprintf('t%d', i)) = t_id(i);
                end
            end
            % Convert to cell so we it's not a pain to debug
            self.times = num2cell(times);

            % Create PZ's for error
            error_pos = [];
            error_vel = [];
            if ~(isempty(self.ultimate_bound) && isempty(self.k_r))
                error_pos(self.num_q) = armour.pz_roahm.polyZonotope_ROAHM();
                error_vel(self.num_q) = armour.pz_roahm.polyZonotope_ROAHM();
                for i = 1:self.num_q
                    e_id = length(fieldnames(self.id_map)) + 1;
                    error_pos(i) = armour.pz_roahm.polyZonotope_ROAHM(0, self.ultimate_bound(i)/self.k_r, [], 1, e_id);
                    self.id_map.(sprintf('e_pos%d', i)) = e_id;
                    error_vel(i) = armour.pz_roahm.polyZonotope_ROAHM(0, 2*self.ultimate_bound(i), [], 1, e_id+1);
                    self.id_map.(sprintf('e_vel%d', i)) = e_id+1;
                end
            end
            % Convert to cell so we it's not a pain to debug
            self.error_pos = num2cell(error_pos);
            self.error_vel = num2cell(error_vel);
        end
    end

    % Generate a new JRS
    methods (Access=protected)
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Wraps create_jrs_online
        % Returns JRSInstance
        function reachableSet = generateReachableSet(self, robotState)
            % The way I choose to wrap this is not the way it has to be
            % done. I choose to have it create the object first, then
            % initialize the remaining properties from this loaded data.
            rs = armour.reachsets.JRSInstance;

            % compat
            traj_type_adapt = self.traj_type;
            if strcmp(traj_type_adapt, 'piecewise')
                traj_type_adapt = 'orig';
            end
            
            self.vdisp("Generating joint reachable set!", "INFO")

            self.vdisp("The following message is from create_jrs_online", 'INFO');
            % Generate it online as per the original implementation
            [rs.q_des, rs.dq_des, rs.ddq_des,            ...
                rs.q, rs.dq, rs.dq_a, rs.ddq_a,             ...
                rs.R_des, rs.R_t_des, rs.R, rs.R_t,         ...
                rs.jrs_info] = armour.legacy.create_jrs_online(  ...
                    robotState.position,               ...
                    robotState.velocity,           ...
                    robotState.acceleration,          ...
                    self.joint_axes, ...
                    self.taylor_degree,         ...
                    traj_type_adapt,                     ...
                    self.add_ultimate_bound, ...
                    self.controller);
            
            % Initialize this particular instance and return
            rs.initialize(self.traj_type);
            % Return the 1 RS that we have.
            reachableSet.rs = rs;
            reachableSet.id = 1;
        end

        function jrs = gen_JRS(self, q_in, qd_in, extras)
            arguments
                self
                q_in
                qd_in
                extras.qdd_in = []
                extras.taylor_degree = 1
                extras.make_gens_independent = true
            end
            q_in = q_in(:);
            qd_in = qd_in(:);
            qdd_in = extras.qdd_in(:);

            % Get the reference trajectory
            switch self.traj_type
            case 'piecewise'
                param_range = min(max(pi/24, abs(qd_in/3)), pi/3);
                param_range = [-param_range, param_range];
                [q_ref, qd_ref, qdd_ref] = genPiecewiseTrajectory(self.num_q, q_in, qd_in, self.tplan, self.tfinal, self.base_params, param_range, self.times);
            end

            % Generate the reference rotatotopes
            for t_idx=self.num_t:-1:1
                for i=self.num_q:-1:1
                    [R_ref{t_idx, i}, R_t_ref{t_idx, i}] = armour.pz_roahm.get_pz_rotations_from_q(q_ref{t_idx, i}, self.joint_axes(:,i), extras.taylor_degree);
                end
            end

            % Add tracking error if provided
            if ~(isempty(self.error_pos) || isempty(self.error_vel))
                for t_idx=self.num_t:-1:1
                    for i=self.num_q:-1:1
                        q{t_idx, i} = q_ref{t_idx, i} + self.error_pos{i};
                        qd{t_idx, i} = qd_ref{t_idx, i} + self.error_vel{i};
                        qd_aux{t_idx, i} = qd_ref{t_idx, i} + self.k_r*self.error_pos{i};
                        qdd_aux{t_idx, i} = qdd_ref{t_idx, i} + self.k_r*self.error_vel{i};
                        [R{t_idx, i}, R_t{t_idx, i}] = armour.pz_roahm.get_pz_rotations_from_q(q{t_idx, i}, self.joint_axes(:,i), extras.taylor_degree);
                    end
                end
            end

            % Make independence if requested
            if extras.make_gens_independent
                for t_idx=self.num_t:-1:1
                    for i=self.num_q:-1:1
                        q_ref{t_idx, i} = remove_dependence_and_compress(q_ref{t_idx, i}, self.k_ids);
                        qd_ref{t_idx, i} = remove_dependence_and_compress(qd_ref{t_idx, i}, self.k_ids);
                        qdd_ref{t_idx, i} = remove_dependence_and_compress(qdd_ref{t_idx, i}, self.k_ids);
                        R_ref{t_idx, i} = remove_dependence_and_compress_mat(R_ref{t_idx, i}, self.k_ids);
                        R_t_ref{t_idx, i} = remove_dependence_and_compress_mat(R_t_ref{t_idx, i}, self.k_ids);
                    end
                end
                if ~(isempty(self.error_pos) || isempty(self.error_vel))
                    for t_idx=self.num_t:-1:1
                        for i=self.num_q:-1:1
                            q{t_idx, i} = remove_dependence_and_compress(q{t_idx, i}, self.k_ids);
                            qd{t_idx, i} = remove_dependence_and_compress(qd{t_idx, i}, self.k_ids);
                            qd_aux{t_idx, i} = remove_dependence_and_compress(qd_aux{t_idx, i}, self.k_ids);
                            qdd_aux{t_idx, i} = remove_dependence_and_compress(qdd_aux{t_idx, i}, self.k_ids);
                            R{t_idx, i} = remove_dependence_and_compress_mat(R{t_idx, i}, self.k_ids);
                            R_t{t_idx, i} = remove_dependence_and_compress_mat(R_t{t_idx, i}, self.k_ids);
                        end
                    end
                end
            end

            % Filler if we don't have error
            if (isempty(self.error_pos) || isempty(self.error_vel))
                q = q_ref;
                qd = qd_ref;
                qd_aux = qd_ref;
                qdd_aux = qdd_ref;
                R = R_ref;
                R_t = R_t_ref;
            end

            jrs = struct;
            jrs.q_ref = q_ref;
            jrs.qd_ref = qd_ref;
            jrs.qdd_ref = qdd_ref;
            jrs.q = q;
            jrs.qd = qd;
            jrs.qd_aux = qd_aux;
            jrs.qdd_aux = qdd_aux;
            jrs.R_ref = R_ref;
            jrs.R_t_ref = R_t_ref;
            jrs.R = R;
            jrs.R_t = R_t;
        end
    end
end

function [q_ref, qd_ref, qdd_ref] = genPiecewiseTrajectory(num_q, q0, qd0, tplan, tfinal, base_params, param_range, times)
    arguments
        num_q
        q0
        qd0
        tplan
        tfinal
        base_params
        param_range
        times
    end

    for i = num_q:-1:1
        param{i} = 0.5 * (base_params{i} + 1) * (param_range(i,2) - param_range(i,1)) + param_range(i,1);
    end

    % Compute key constants
    stopping_time = tfinal - tplan;

    % Peak values (first part of the traj)
    for i=num_q:-1:1
        qpeak{i} = q0(i) + qd0(i)*tplan + 0.5*param{i}*(tplan^2);
        qdpeak{i} = qd0(i) + param{i}*tplan;
    end

    % Braking values (second part of the traj)
    for i=num_q:-1:1
        stopping_qdd{i} = (-1) * qdpeak{i} * (1.0/stopping_time);
        final_q{i} = qpeak{i} + qdpeak{i}*stopping_time + 0.5*stopping_qdd{i}*(stopping_time^2);
    end

    % Create the trajectory
    for idx_t=length(times):-1:1
        t = times{idx_t};
        for i=num_q:-1:1
            if t.c <= tplan
                q_ref{idx_t, i} = q0(i) + qd0(i).*t + 0.5*param{i}.*(t.*t);
                qd_ref{idx_t, i} = qd0(i) + param{i}.*t;
                qdd_ref{idx_t, i} = param{i};
            elseif t.c > tplan && t.c <= tfinal
                tshift = (t-tplan);
                q_ref{idx_t, i} = qpeak{i} + qdpeak{i}.*tshift + 0.5*stopping_qdd{i}.*(tshift.*tshift);
                qd_ref{idx_t, i} = qdpeak{i} + stopping_qdd{i}.*(tshift);
                qdd_ref{idx_t, i} = stopping_qdd{i};
            else
                q_ref{idx_t, i} = final_q{i};
                qd_ref{idx_t, i} = 0;
                qdd_ref{idx_t, i} = 0;
            end
        end
    end
end

function B = remove_dependence_and_compress(A, k_id)
import armour.pz_roahm.*
	k_id_idx = any(A.id == k_id(:).', 2);
	k_slc_idx = (any(A.expMat(k_id_idx, :) ~= 0, 1) & all(A.expMat(~k_id_idx, :) == 0, 1)); % should only be one!
	if length(find(k_slc_idx)) > sum(k_id_idx)
		error('There should only be one fully-k-sliceable generator');
	end
	B = polyZonotope_ROAHM(A.c, A.G(k_slc_idx), sum(abs(A.G(~k_slc_idx))) + sum(abs(A.Grest)), A.expMat(k_id_idx, k_slc_idx), A.id(k_id_idx));
	% B = polyZonotope_ROAHM(A.c, A.G(k_slc_idx), [A.G(~k_slc_idx), A.Grest], A.expMat(k_id_idx, k_slc_idx), k_id);
end

function B = remove_dependence_and_compress_mat(A, k_id)
import armour.pz_roahm.*
	k_id_idx = any(A.id == k_id(:).', 2);
	k_slc_idx = (any(A.expMat(k_id_idx, :) ~= 0, 1) & all(A.expMat(~k_id_idx, :) == 0, 1)); % should only be one!
% 	if length(find(k_slc_idx)) > 1
% 		error('There should only be one fully-k-sliceable generator');
% 	end
	B = matPolyZonotope_ROAHM(A.C, A.G(:, :, k_slc_idx), cat(3, A.G(:, :, ~k_slc_idx), A.Grest), A.expMat(k_id_idx, k_slc_idx), A.id(k_id_idx));
end