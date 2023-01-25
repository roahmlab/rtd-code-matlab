classdef ArmourAgent < rtd.sim.world.WorldEntity & handle
% The Agent with the robust controller for ARMOUR
% Left to do are the helper safety check functions like check input limits
% and glueing this together
    % Required Abstract Properties
    properties
        %%%%%%%%%%%%%%%%%%%
        % Data Components %
        %%%%%%%%%%%%%%%%%%%
        
        % Core data to describe the agent.
        % Should be mostly invariant.
        info = ArmourAgentInfo.empty()
        
        % The changing values and their history that fully describe the
        % state of the agent at any given (valid) point in time
        state = ArmourAgentState.empty()
    end

    % Specific for this entity
    properties
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Prerequisite Utility Components %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % How do we calculate the forward and inverse kinematics for the
        % agent?
        kinematics
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Behavior + Data Components %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % What is the control input(s) to this agent for a chosen
        % trajectory or similar?
        % This also stores relevant controller info and history of
        % trajectories if applicable.
        controller
        
        % What is the evolution of the agent's state given some control
        % input?
        % This stores the history of the control inputs used for the
        % dynamics.
        dynamics
        
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % Behavior Components %
        %%%%%%%%%%%%%%%%%%%%%%%
        
        % What is the geometry that is actually relevant to the collision
        % system or is the agent in collision?
        collision
        
        % What is the resulting visual geometry or rendered result for
        % whatever visualization system we are using?
        visual
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Other Utility Components %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % None
        
    end
    
    methods (Static)
        function options = defaultoptions()
            options = rtd.sim.world.WorldEntity.baseoptions();
            
            % These are the names for the default components
            components.info = 'ArmourAgentInfo';
            components.state = 'ArmourAgentState';
            components.kinematics = 'ArmKinematics';
            components.controller = 'ArmourController';
            components.dynamics = 'ArmourDynamics';
            components.collision = 'ArmourPatchCollision';
            components.visual = 'ArmourPatchVisual';
            options.components = components;
        end
        
        function self = from_options(robot, params, options)
            a = ArmourAgentInfo(robot, params, options.component_options.info);
            self = ArmourAgent(a, optionsStruct=options);
        end
    end
    
    methods
        % WIP
        function self = ArmourAgent(info, components, optionsStruct, options)
            arguments
                info
                components.state = []
                components.kinematics = []
                components.controller = []
                components.dynamics = []
                components.collision = []
                components.visual = []
                optionsStruct.optionsStruct struct = struct()
                options.components
                options.component_options
                options.component_logLevelOverride
                options.verboseLevel
                options.name
            end
            % Get override options based on provided components
            components.info = info;
            override_options = rtd.sim.world.WorldEntity.get_componentOverrideOptions(components);

            % Merge all options
            self.mergeoptions(optionsStruct.optionsStruct, options, override_options);
            
            % Set components and (Re)construct dependent components for
            % consistency
            self.info = info;
            self.construct_component('state', self.info);
            self.construct_component('kinematics', self.info, self.state);
            self.construct_component('controller', self.info, self.state);
            self.construct_component('dynamics', self.info, self.state, self.controller);
            self.construct_component('collision', self.info, self.state, self.kinematics);
            self.construct_component('visual', self.info, self.state, self.kinematics);
            
            % Reset
            self.reset()
        end
        
        % Reset all components.
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.component_options
                options.component_logLevelOverride
                options.verboseLevel
                options.name
            end
            % Perform an internal update, then merge in options
            self.getoptions();
            options = self.mergeoptions(optionsStruct, options);
            
            % reset all components
            self.reset_components()
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
        end
        
        % Update this agent's state by t_move
        function results = update(self, t_move)
            % First move
            self.dynamics.move(t_move);
            results.success = true; %hardcoded for now
            % Then run post-movement checks
            t_check_step = 0.01;
            results.checks.joint_limits = self.state.joint_limit_check(t_check_step);
            results.checks.control_inputs = self.dynamics.controller_input_check(t_check_step);
            results.checks.ultimate_bound = self.controller.ultimate_bound_check(t_check_step, self.dynamics.controller_log);
        end
    end
end