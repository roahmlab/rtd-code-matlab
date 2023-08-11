 %% get agent info
 classdef RefineAgentInfo < rtd.entity.components.BaseInfoComponent & rtd.util.mixins.Options & handle
     properties
        dimension = 3;
        desired_initial_condition = [0; 0; 0; 1; 0; 0; 1; 0; 0; 0];
        %vehicle parameter in simulation
        m %mass in kg
        lf %
        lr
        mu_bar %coefficient of friction
        Caf1 %torque
        Caf2 %torque
        Car1
        Car2 
        Izz %moment of inertia along z axis
        rw %radius of wheels
        
        grav_const
        l
        %% Complicated LLC to cancel out dynamics; Front Wheel drive
        %Refine coltroller parameter in simulation
        Ku %control gain
        Kh %control gain
        Kr %control gain
        kappaPU 
        kappaIU
        phiPU
        phiIU
        kappaP 
        kappaI
        phiP
        phiI

        Mu
        Mr
        name
        
        Cus
        u_cri %critical velocity
        
        max_Fy_uncertainty
        max_Fx_uncertainty
        max_Fx_uncertainty_braking

        default_footprint
        n_states
        n_inputs
        stopping_time
        sensor_radius
        plot_data struct
        make_footprint_plot_data
        make_arrow_plot_data

     end

      methods (Static)
        function options = defaultoptions()
            % Configurable options for this component
            options.M_min_eigenvalue = 0.002; % arbitrary value
            options.gravity = [0 0 -9.81];
            options.joint_velocity_limits = [];
            options.joint_torque_limits = [];
            options.transmission_inertia = [];
            options.buffer_dist = 0;
        end
    end
     
     methods
         function A = RefineAgentInfo(varargin)

            A.name = 'highway_cruiser' ;
            
            A.default_footprint = [4.8 2.2]; 
            A.n_states = 10 ;
            A.n_inputs = 6 ; % ud vd rd dud dvd drd
            A.stopping_time = 50 ; %not used
            A.sensor_radius = 400 ;

            % set up plot data
            A.plot_data = struct();
            A.plot_data.trajectory = [] ;
            A.plot_data.footprint = [] ;
            A.plot_data.arrow = [] ;
            
            % set up footprint and arrow for plot
            A.make_footprint_plot_data() ;
            A.make_arrow_plot_data() ;
            
            % reset time, states, and inputs, just in case
%             A.reset() ;
            
            
%             % create agent
%             A@RTD_agent_2D('name',name,...
%             'footprint',default_footprint,...
%             'n_states',n_states,'n_inputs',n_inputs,...
%             'stopping_time',stopping_time,'sensor_radius',sensor_radius,varargin{:});

            load('my_const.mat')

            A.m  = m;
            A.lf = lf;
            A.lr = lr;
            A.l = lf+lr;
            A.grav_const = grav_const;
            A.Caf1 = Caf1;
            A.Caf2 = Caf2;
            A.Car1 = Car1;
            A.Car2 = Car2;
            A.mu_bar = mu_bar;
            A.Izz  = Izz;
            A.rw = rw;
            A.Mu = Mu;
            A.Mr = Mr;

            %% proposed controller to cancel out dynamics; Front Wheel drive
            A.Ku = Ku;
            A.Kh = Kh;
            A.Kr = Kr;
            A.kappaPU = kappaPU; % kappa_1,u 
            A.kappaIU = kappaIU; % kappa_2,u
            A.phiPU = phiPU;   % phi_1,u
            A.phiIU = phiIU;   % phi_2,u
            A.kappaP = kappaP;  % kappa_1,r 
            A.kappaI = kappaI;    % kappa_2,r
            A.phiP = phiP;      % phi_1,r
            A.phiI = phiI;      % phi_2,r

            A.max_Fy_uncertainty = max_Fy_uncertainty;
            A.max_Fx_uncertainty = max_Fx_uncertainty;
            A.max_Fx_uncertainty_braking = max_Fx_uncertainty_braking;
            A.Cus = m * grav_const * (lr / (A.l * Caf1) - lf / (A.l * Car1));
            A.u_cri = u_really_slow;


         end

         
        function agent_info = get_agent_info(A)
            % call superclass method
%             agent_info = get_agent_info@RTD_agent_2D(A) ;

            agent_info.dimension = A.dimension ;
            agent_info.state = A.state ;
            agent_info.position = A.state(A.position_indices,:) ;
            agent_info.position_indices = A.position_indices ;
            agent_info.time = A.time ;
            agent_info.sensor_radius = A.sensor_radius ;
            agent_info.n_inputs = A.n_inputs ;
            agent_info.n_states = A.n_states ;

             % additional fields
            agent_info.heading_index = A.heading_index ;
            agent_info.desired_time = A.desired_time ;
            agent_info.desired_input = A.desired_input ;
            agent_info.desired_trajectory = A.desired_trajectory ;
            agent_info.heading_index = A.heading_index ;
            agent_info.footprint = A.footprint ;
            agent_info.footprint_vertices = A.footprint_vertices ;
        end
        
%         function reset(A,state)
%             if nargin < 2
%                 
%                 A.desired_time = zeros(1,0);
%                 A.desired_input = zeros(2,0);
%                 A.desired_trajectory =zeros(2,0);
%                 reset@RTD_agent_2D(A,[A.desired_initial_condition]) ;
%             else
%                 reset@RTD_agent_2D(A,state) ;
%             end
%         end

        %% reset
        function reset(A,state)
            if nargin < 2
                A.desired_time = zeros(1,0);
                A.desired_input = zeros(2,0);
                A.desired_trajectory =zeros(2,0);
                state = [A.desired_initial_condition];
            end
            
            % do the reset
            A.state = zeros(A.n_states,1) ;
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ;
            A.input_time = 0 ;
            
            % reset the state
            switch length(state)
                case A.n_states
                    A.state = state ;
%                 case 2
%                     A.state(A.position_indices) = state ;
%                 case 3
%                     A.state([A.position_indices,A.heading_index]) = state ;
                otherwise
                    error(['The provided state has an incorrect number of elements!',...
                        ' Please provide ',...
                        ' an n_states-by-1 full state vector.'])
            end
            
            A.desired_trajectory = [];
            A.desired_input = [];
            A.desired_time = [];
        end

        

        
        
     end
 end
