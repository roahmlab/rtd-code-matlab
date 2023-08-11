classdef RefineDynamics < rtd.entity.components.BaseDynamicsComponent & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle

    methods
        function [dzdt, Fxwf, Fywf, Fywr ,delta ,w_cmd, v, r, Fxwr]= RefineDynamics(A,t,z,T,U,Z) % extra output used

            
            h = z(3);
            u = z(4);
            v = z(5);
            r = z(6);
          

            if  u > A.u_cri
                    [delta, w_cmd, cur_r_err, cur_h_err, cur_u_err] = A.FL_LLC(t,z,T,U,Z);
            else 
                    [delta, w_cmd, v, r, cur_u_err] = A.Low_Spd_LLC(t,z,T,U,Z);
                     cur_r_err= 0;
                     cur_h_err = 0;
            end

            uddot = interp1(T,U(4,:),t,'linear');
            vf = v + A.lf*r;
            vr = v - A.lr*r;
            uv_wf = rotmat(-delta)*[u;vf];
            uv_wr = [u; vr];
            alphar = atan( uv_wr(2) ./ max(u, A.u_cri*1.1)); % modify denominator for numerical stability
            Fywr = -A.Car1*tanh(A.Car2*alphar);
            if uddot <= 0
                lambda_f = (A.rw * w_cmd - uv_wf(1)) / max(uv_wf(1), 0.01);
            else
                lambda_f = (A.rw * w_cmd - uv_wf(1)) / max(A.rw * w_cmd, 0.01);
            end
            alphaf = atan( uv_wf(2) ./ (uv_wf(1) + sign(uv_wf(1)+0.001)*0.01));
            Cbf = A.m * A.grav_const * A.lr / A.l * A.mu_bar;
            Fxwf = Cbf*lambda_f;
            Fxwr = 0;
            Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
            
            du =  v*r + (cos(delta)*Fxwf-sin(delta)*Fywf + Fxwr)/A.m;
            dv = - u*r + (sin(delta)*Fxwf+cos(delta)*Fywf + Fywr)/A.m ;
            torque =  A.lf*(sin(delta)*Fxwf+cos(delta)*Fywf) - A.lr * Fywr;
            dr = torque/A.Izz;
            
            
             if u > A.u_cri
                dzdt = [u*cos(h)-v*sin(h);
                u*sin(h)+v*cos(h);
                r;
                du;
                dv;
                dr;
                0;
                cur_r_err^2;
                cur_h_err^2;
                cur_u_err^2];
             else 
                dzdt = [u*cos(h)-v*sin(h);
                u*sin(h)+v*cos(h);
                r;
                du;
                0;
                0;
                0;
                0;
                0;
                cur_u_err^2];
             end
        
        
            if any(isnan(dzdt))
                error("nan appeared in dzdt = "+num2str(dzdt))
            end
            
        end

        %% integrator options
        function [tout,zout] = integrator(A,fun,tspan,z0)
            switch A.integrator_type
                case 'ode45'
                    [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:)) ;
                case 'ode113'
                    [tout,zout] = ode113(@(t,z) fun(t,z),tspan,z0(:)) ;
                case 'ode1'
                    dt = A.integrator_time_discretization ;
                    tout = tspan(1):dt:tspan(end) ;
                    zout = ode1(@(t,z) fun(t,z),tout,z0(:)) ;
                case {'ode4','RK4'}
                    dt = A.integrator_time_discretization ;
                    tout = tspan(1):dt:tspan(end) ;
                    if tout(end) ~= tspan(end)
                        tout = [tout, tspan(end)] ;
                    end
                    zout = ode4(@(t,z) fun(t,z),tout,z0(:)) ;
                otherwise
                    error('Please set A.integrator_type to either ode45 or ode4')
            end
            tout = tout(:)' ;
            zout = zout' ;
        end
%%
        function stop(A,t_stop)
            if nargin < 2
                t_stop = A.stopping_time ;
            end
            
            T_input = 0:0.1:t_stop ;
            N_t = length(T_input) ;
            
            pose =  A.state([A.position_indices,A.heading_index],end) ;
            stopped_state = [pose ; zeros(A.n_states-3,1)] ;
            
            Z_desired = repmat(stopped_state,1,N_t) ;
            
            U_input = zeros(A.n_inputs,N_t) ;
            
            A.move(t_stop,T_input,U_input,Z_desired) ;
        end
        

        %%
         function commit_move_data(A,T_state,Z_state,T_used,U_used,~)
            % method: commit_move_data(T_state,Z_state,T_input,U_input,Z_input)
            %
            % After moving the agent, commit the new state and input
            % trajectories, and associated time vectors, to the agent's
            % state, time, input, and input_time properties.
            
            % update the state, time, input, and input time
            A.state = [A.state, Z_state(:,2:end)] ;
            A.time = [A.time, A.time(end) + T_state(2:end)] ;
            A.input_time = [A.input_time, A.input_time(end) + T_used(2:end)] ;
            A.input = [A.input, U_used(:,1:end-1)] ;
         end

         %%
         function [T,U,Z] = move_setup(A,t_move,T_ref,U_ref,Z_ref)
            % method: [T,U,Z] = move_setup(A,t_move,T_ref,U_ref,Z_ref)
            %
            % Given an amount of time t_move for the agent to move, and a
            % reference time vector, output a time vector to actually use
            % during the call to move the agent.
            
            % make sure the reference time is a row vector
            T_ref = T_ref(:)' ;
            
            % make sure the reference time is unique, and only get those
            % rows of U_ref and Z_ref
            [T_ref,unique_idxs,~] = unique(T_ref,'stable') ;
            U_ref = U_ref(:,unique_idxs) ;
            if ~isempty(Z_ref)
                Z_ref = Z_ref(:,unique_idxs);
            end
            
            % sanity check the timing
            if T_ref(1) > 0
                warning(['Provided reference time does not start from 0! ',...
                    'The reference trajectory will be padded with the ',...
                    'first reference input and reference state at time 0'])
                T_ref = [0, T_ref(:)'] ;
                U_ref = [U_ref(:,1), U_ref] ;
                Z_ref = [Z_ref(:,1), Z_ref] ;
            end
            
            if T_ref(end) < t_move
                warning(['Provided input time vector is shorter than the ',...
                    'desired motion time! The agent will still be ',...
                    'moved for the duration t_move. The reference ',...
                    'time, input, and trajectory will be padded ',...
                    'to be of duration t_move.'])
                
                T_ref = [T_ref(:)', t_move] ;
                U_ref = [U_ref, U_ref(:,end)] ;
                Z_ref = [Z_ref, Z_ref(:,end)] ;
            end
            
            % get the amount of time to actually move the agent
            T_log = T_ref < t_move ;
            
            % make sure t_move itself is included in the time vector (this
            % solves a nasty bug that could introduce NaNs in the agent's
            % state unintentionally!)
            T = [T_ref(T_log), t_move] ;
            
            % interpolate the reference input and trajectory to pass to the
            % agent's move method
            if nargin < 5 || isempty(Z_ref)
                U = match_trajectories(T,T_ref,U_ref) ;
                Z = [] ;
            else
                [U,Z] = match_trajectories(T,T_ref,U_ref,T_ref,Z_ref) ;
            end
         end

          %% move
        function move(A,t_move,T_ref,U_ref,Z_ref)
            % method: move(t_move,T_ref,U_ref,Z_ref)
            %
            % Moves the agent for the duration t_move using the nominal
            % inputs U_ref and nominal trajectory Z_ref that are indexed by
            % the nominal time T_ref.
            %
            % This method assumes that the input is zero-order hold, and
            % the input corresponding to the last time index is zero; this
            % is why the last old input is discarded when the input is
            % updated. Similarly, this method assumes that the nominal time
            % starts at 0.
            
            A.vdisp('Moving!',5)
            
            % set up default reference trajectory
            if nargin < 5
                Z_ref = [] ;
            end
            
            % get the time, input, and reference trajectory to use for
            % moving the agent
            [T_used,U_used,Z_used] = A.move_setup(t_move,T_ref,U_ref,Z_ref) ;
            
            % get the current state
            zcur = A.state(:,end) ;
            
            % call the ode solver to simulate agent
            [tout,zout] = A.integrator(@(t,z) A.dynamics(t,z,T_ref,U_ref,Z_ref),...
                                       [0 t_move], zcur) ;
            
            A.commit_move_data(tout,zout,T_used,U_used,Z_used) ;
        end
    end

end
