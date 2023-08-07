  %% dynamics
        function [dzdt, Fxwf, Fywf, Fywr ,delta ,w_cmd, v, r, Fxwr]= dynamics(A,t,z,T,U,Z) % extra output used

            
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