classdef Trajectory_reference < rtd.planner.trajectory.Trajectory

    properties

        AH
        K
        agent_state
        trajectory
        u0
        h0
        au
        ay
        u0_goal
        manu_type
        t0_offset

%         ref_Z = [];
%         proposed_ref_Z = [];
%         t_real_start = [];
%         t_proposed_start = [];
        
    end

 properties (Constant)
        vectorized = true; % Set the value based on whether your subclass supports time vector commands
    end

    methods
        function traj=Trajectory_reference(AH, K, agent_state)% AgentHelper, K=[Au,Ay,t0_idx,Manu_type], agent_state = [x_curr,y_curr,h_curr,u_curr]
           
            arguments
                AH 
                K
                agent_state
            end

            self.AH = AH;
            self.K = K;
            self.agent_state = agent_state;

            %find T,U,Z
            u_cur = agent_state(4) ;
            y_cur = agent_state(2) ;
            x_cur = agent_state(1) ;
            Au = K(1);
            Ay = K(2);
            t0_idx = K(3);
            AH.t_move=3;
            t0 = (t0_idx-1)*AH.t_move;
            type_manu = K(4);
            if type_manu == 3 % 1: speed change. 2: direction change. 3: lane change
                [T, U,Z] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            else
                [T, U,Z] = sin_one_hump_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            end
            self.trajectory = Z;
        end
 
        function valid = validate(self, throwOnError)
            arguments
                self rtd.planner.trajectory.Trajectory
                throwOnError(1,1) logical = false
            end
            
            % Check if trajectoryParams, trajOptProps, and startState are empty
            valid = ~isempty(self.trajectoryParams) && ~isempty(self.trajOptProps) && ~isempty(self.startState);
            
            % Additional validation logic specific to the Trajectory class
            
            % Throw error if invalid and throwOnError is true
            if ~valid && throwOnError
                errMsg = MException('Trajectory:InvalidTrajectory', 'The trajectory object is not fully parameterized!');
                throw(errMsg);
            end
        end

        function setParameters(traj, au, ay, u0_goal, manu_type, u0, t0_offset, h0)
            traj.au = au;
            traj.ay = ay;
            traj.u0_goal = u0_goal;
            traj.manu_type = manu_type;
            
            if nargin > 5
                traj.u0 = u0;
            end
            
            if nargin > 6
                traj.t0_offset = t0_offset;
            end
            
            if nargin > 7
                traj.h0 = h0;
            end
        end

        function command = getCommand(self, time)
            arguments
                self Trajectory
                time(1,:) double
            end
        
            % Do a parameter check and time check, and throw if anything is
            % invalid.
            self.validate(true);
            t_shifted = time - self.startState.time;
            if t_shifted < 0
                ME = MException('Trajectory:InvalidTrajectory', ...
                    'Invalid time provided to Trajectory');
                throw(ME)
            end
        
            % Perform the necessary calculations using the provided parameters
            ud = self.Ud(t_shifted);
            vd = self.Vd(t_shifted);
            rd = self.Rd(t_shifted);
            ud_dot = self.UdDot(t_shifted);
            vd_dot = self.VdDot(t_shifted);
            rd_dot = self.RdDot(t_shifted);
            hd = self.Hd(t_shifted);
        
            % Generate the output.
            command = struct('ud', ud, 'vd', vd, 'rd', rd, 'ud_dot', ud_dot, 'vd_dot', vd_dot, 'rd_dot', rd_dot, 'hd', hd);
        end
    end

     %% All Private methods

    methods(Access=private)
        function cti = ComputeInfo(obj)
            cti.delta_spd = max(obj.au - RtdConsts.kUReallySlow, 0);
            if(obj.OneHump())
                 cti.t_to_use = RtdConsts.kTpkDir; 
            else
                 cti.t_to_use = RtdConsts.kTpk;
            end
            
            cti.tbrk1 = cti.delta_spd / RtdConsts.kAMax;
            cti.tbrk2 = 1.0;
            cti.t_eval_max = cti.t_to_use + cti.tbrk1 + cti.tbrk2;
        end

        function ud = Ud(obj, t)
            if t <= obj.T1Max()
                ud = ((obj.au - obj.u0) / obj.TToUse()) * t + obj.u0;
            elseif t <= obj.T2Max()
                ud = obj.au - ((obj.au - Constants.UReallySlow) / obj.TBrk1()) * (t - obj.TToUse());
            else
                ud = 0.0;
            end
        end
        
        function vd = Vd(obj, t)
            vd = obj.Rd(t) / 13.0;
        end
        
        function rd = Rd(obj, t)
            if t <= obj.T1Max()
                if obj.OneHump()
                    rd = obj.ay * 0.5 * (sin(t * Constants.FourThirdPi - Constants.HalfPi) + 1.0);
                else
                    rd = -obj.ay * exp(-2.7 * (t - 1.5).^2) * (4.0 * t - 6.0);
                end
            else
                rd = 0.0;
            end
        end

        function hd = Hd(obj, t)
            % Compute hd at time t
            % TODO +h0
            if t <= obj.T1Max()
                if obj.OneHump()
                    ret_val = obj.ay * (t / 2 - (3 / (8 * pi)) * sin(obj.kFourThirdPi * t));
                else
                    ret_val = -obj.ay * (-20 / 27 * exp(-27 / 40 * Square(3 - 2 * t)));
                end
                hd = ToAngle(ret_val + obj.h0);
            else
                % hd should remain constant during braking
                hd = obj.Hd(obj.T1Max());
            end
        end
    
        function uddot = UdDot(obj, t)
            % Compute uddot at time t
            eval_times = FiniteDiffTimes(obj,t);
            delta = (obj.Ud(eval_times.Ub()) - obj.Ud(eval_times.Lb()));
            uddot = delta / obj.kFiniteDiffTime;
        end
        
        function vddot = VdDot(obj, t)
            % Compute vddot at time t
            eval_times = FiniteDiffTimes(obj,t);
            delta = (obj.Vd(eval_times.Ub()) - obj.Vd(eval_times.Lb()));
            vddot = delta / obj.kFiniteDiffTime;
        end
        
        function rddot = RdDot(obj, t)
            % Compute rddot at time t
            eval_times = FiniteDiffTimes(obj,t);
            delta = (obj.Rd(eval_times.Ub()) - obj.Rd(eval_times.Lb()));
            rddot = delta / obj.kFiniteDiffTime;
        end

    
        function t_brk1 = TBrk1(obj)
            t_brk1 = obj.ComputeInfo().tbrk1;
        end
        
        function t_brk2 = TBrk2(obj)
            t_brk2 = obj.ComputeInfo().tbrk2;
        end
        
        function t_to_use = TToUse(obj)
            t_to_use = obj.ComputeInfo().t_to_use;
        end
        
        function delta_spd = DeltaSpd(obj)
            delta_spd = obj.ComputeInfo().delta_spd;
        end
        
        function t_eval_max = TEvalMax(obj)
            t_eval_max = obj.ComputeInfo().t_eval_max;
        end
        
        function t1_max = T1Max(obj)
            t1_max = obj.TToUse();
        end
        
        function t2_max = T2Max(obj)
            t2_max = obj.T1Max() + obj.TBrk1();
        end
        
        function t3_max = T3Max(obj)
            t3_max = obj.T2Max() + obj.TBrk2();
        end
        
         %def to angle function here
        function angle_ = ToAngle(theta)
            if(~(isfinite(theta)))
                angle_ = 0;
            else
                angle_ = angle(cos(theta) + 1i * sin(theta));
            end
        
        end
        %define finite diff time function here
        function interval = FiniteDiffTimes(obj,t)
            kFiniteDiffTime = 1.0e-6;
            kHalfDiffTime = kFiniteDiffTime / 2.0;
            centered = [t - kHalfDiffTime, t + kHalfDiffTime];
            left_biased = [t - kFiniteDiffTime, t];
            right_biased = [t, t + kFiniteDiffTime];
            if TimeIntervalisOK(obj,centered)
                interval = centered;
            elseif TimeIntervalisOK(obj,left_biased)
                interval = left_biased;
            elseif TimeIntervalisOK(obj,right_biased)
                interval = right_biased;
            else
                error('Nothing is OK!!');
            end
        end

        function isOK = TimeIntervalisOK(obj,time_interval)
            min = TimeinRange(obj,time_interval(1));
            max = TimeinRange(obj,time_interval(2));
            bounds_in_range = TimeinSameRange(obj,time_interval(1),time_interval(2));
            isOK = min && max && bounds_in_range;
        end
        
        function t = TimeinRange(obj,t1)
            if(t1 >=0 && t1<= TEvalMax(obj))
                t = 1;
            end
        end

        function t = TimeinSameRange(obj,t1,t2)
            if(Range(obj,t1)==Range(obj,t2))
                t = 1;
            end
        end

        function r = Range(obj,t)
            if(t<0)
                r = -1;
            elseif(t<T1MAX(obj))
                r = 1;
            elseif(t<T2MAX(obj))
                r = 2;
            elseif(t<T3MAX(obj))
                r=3;
            else
                r=4;
            end
        end

        function t1max = T1MAX(obj)
            t1max = TToUse(obj);
        end

        function t1max = T2MAX(obj)
            t1max = T1MAX(obj) + TBrk1(obj);
        end

        function t1max = T3MAX(obj)
            t1max = T2MAX(obj) + TBrk2(obj);
        end

        %define square function here
        function sq = Square(x)
            sq = x*x;
        end

    end

end