classdef ArmAgent < handle & NamedClass & UUIDbase
% Generic Arm Agent Entity
    properties
        %%%%%%%%%%%%%%%%%%%
        % Data Components %
        %%%%%%%%%%%%%%%%%%%
        
        % Core data to describe the agent.
        % Should be mostly invariant.
        info
        
        % The changing values and their history that fully describe the
        % state of the agent at any given (valid) point in time
        state
        
        
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
        
        
        %%%%%%%%%%%%%%%%%%%%%%
        % Utility Components %
        %%%%%%%%%%%%%%%%%%%%%%
        
        % How do we calculate the forward and inverse kinematics for the
        % agent?
        kinematics
    end
    
    methods
        function self = ArmAgent()
            
        end
        
        % Reset all components.
        function reset(self, state)
            % TODO Improve generalization
            % TODO Allow reset with an options struct
            % TODO Create an options method
            self.info.reset();
            self.state.reset(state);
            self.controller.reset();
            self.dynamics.reset();
            self.collision.reset();
            self.visual.reset();
            self.kinematics.reset();
        end
        
        % Get all the options used for initialization of the robot
        function agent_options = getOptions(self)
            % TODO autogenerate a struct of all options, which can be used
            % for initialization of this agent as well
            agent_options = self;
        end
        
        % Pass the trajectory to the controller
        % TODO Decide on relevance
        function setTrajectory(self, trajectory)
            self.controller.setTrajectory(trajectory);
        end
        
        % Move the agent
        % TODO Decide on relevance
        function move(self, t_move)
            self.dynamics.move(t_move);
        end
        
        % Safety checks
        function check(self, t_range)
            % TODO make this addable, and check across? Maybe not (reasons
            % not are obfuscation)
            self.controller.ultimate_bound_check(t_range)
        end
    end
end
        