

classdef Trajectory < Consts
    properties
        t_min
        au
        ay
        u0
        h0
        u0_goal
        manu_type
        t0_offset
    end

    methods
        function obj = Trajectory (au,ay,u0_goal,manu_type,u0,t0_offset,h0)
            if nargin < 5
                u0 = 0.0;
            end
    
            if nargin < 6
                t0_offset = 0.0;
            end
    
            if nargin < 7
                h0 = 0.0;
            end
    
            obj.t_min = 0.0;
            obj.au = max(au,0.5);
            obj.ay = ay;
            obj.u0_goal = u0_goal;
            obj.manu_type = manu_type;
            obj.u0 = u0;
            obj.t0_offset = t0_offset;
            obj.h0 = h0;
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
    
        function value = EvalAt (obj,t)
            t = max(t,obj.t_min);
            ud = obj.Ud(t);
            vd = obj.Vd(t);
            rd = obj.Rd(t);
            ud_dot = obj.UdDot(t);
            vd_dot = obj.VdDot(t);
            rd_dot = obj.RdDot(t);
            hd = obj.Hd(t);
            value = struct('ud',ud,'vd',vd,'rd',rd,'ud_dot',ud_dot,'vd_dot',vd_dot,'rd_dot',rd_dot,'hd',hd);
        end
    
        function maxTime = MaxTrajTime(obj)
            maxTime = obj.TEvalMax();
        end
    
        function isValid = HasValidType(obj)
            isValid = obj.IsValid(obj.manu_type);
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


    end

    methods (Static)
        function traj = CreateTrajectory(au, ay, u0_goal, manu_type, u0, t0_offset, h0)
            if nargin < 5
                u0 = 0.0;
            end
            if nargin < 6
                t0_offset = 0.0;
            end
            if nargin < 7
                h0 = 0.0;
            end
            traj = Trajectory(au, ay, u0_goal, manu_type, u0, t0_offset, h0);
        end
    end

end

