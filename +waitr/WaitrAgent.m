classdef WaitrAgent < armour.ArmourAgent
% The Agent with the robust controller for ARMOUR
% Left to do are the helper safety check functions like check input limits
% and glueing this together
    methods (Static)
        function options = defaultoptions()
            options = armour.ArmourAgent.defaultoptions();
            
            % These are the names for the default components
            components = options.components;
            components.state = 'waitr.agent.WaitrAgentState';
            components.dynamics = 'waitr.agent.WaitrDynamics';
            options.components = components;
        end
        
        function self = from_options(robot, params, options)
            a = armour.agent.ArmourAgentInfo(robot, params, options.component_options.info);
            self = waitr.WaitrAgent(a, optionsStruct=options);
        end
    end
    
    methods
        % Update this agent's state by t_move
        function results = update(self, t_move)
            update@armour.ArmourAgent(self, t_move);

            % Added post-movement check
            t_check_step = 0.01;
            results.checks.grasp_constraints = self.dynamics.grasp_constraint_check(t_check_step);
        end
    end
end