classdef EEGenerator < rtd.planner.reachsets.ReachSetGenerator
    % ArmTdForwardOccupancy
    % This either encapsulates the reachable sets in memory, or enables the
    % online computation of reachable sets. It acts as a generator for a
    % single instance of ReachableSet
    properties
        cache_max_size = 1
    end
    
    % Additional Properties
    properties
        jrsGenerator
        robot
        u_s
        surf_rad
    end
    methods
        % An example constructor, but can take anything needed
        function self = EEGenerator(robot, jrsGenerator, options)
            arguments
                robot armour.ArmourAgent
                jrsGenerator armour.reachsets.JRSGenerator
                options.u_s(1,1) double = 0.609382421
                options.surf_rad(1,1) double = 0.029
            end
            self.robot = robot;
            self.jrsGenerator = jrsGenerator;
            self.u_s = options.u_s;
            self.surf_rad = options.surf_rad;
        end
    end
    methods (Access=protected)
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns ReachbleSet
        function reachableSet = generateReachableSet(self, robotState)
            % Computes the forward kinematics and occupancy
            
            % First get the JRS (allow the use of a cached value if it
            % exists)
            jrsInstance = self.jrsGenerator.getReachableSet(robotState, ignore_cache=false);
            
            % set up zeros and overapproximation of r
            self.vdisp("Set up zeros for overapproximation", 'TRACE')
            for j = 1:jrsInstance.n_q
                zero_cell{j, 1} = armour.pz_roahm.polyZonotope_ROAHM(0); 
                r{j, 1} = armour.pz_roahm.polyZonotope_ROAHM(0, [], self.robot.controller.ultimate_bound);
            end
            
            self.vdisp("start RNEA")
            for i = 1:jrsInstance.n_t
                self.vdisp("RNEA interval for robust input", 'TRACE')
                [tau_int{i, 1}, f_int{i, 1}, n_int{i, 1}] = ...
                    armour.legacy.dynamics.poly_zonotope_rnea( ...
                        jrsInstance.R{i}, ...
                        jrsInstance.R_t{i}, ...
                        jrsInstance.dq{i}, ...
                        jrsInstance.dq_a{i}, ...
                        jrsInstance.ddq_a{i}, ...
                        true, ...
                        self.robot.info.params.pz_interval);
            end

            % need to add a check somewhere to see if constraint is
            % trivially satisfied

            % iterate through all of the time steps
            for i = 1:jrsInstance.n_t
                
                % only checking one contact joint (for now)
                % ASSUMING SURFACE NORMAL IS POSITIVE Z-DIRECTION

                % ! this depends on the robot urdf being used!
                % for fetch_waiter_Zac.urdf, f_int{i,1}(10), 
                % n_int{i,1}(10) are the forces/moments
                % polyzonotopes at the contact point between 
                % tray and cup.
    
                % extract relevant polyzonotope from f_int{i,1}
                % ! depends on robot urdf!
                contact_poly = f_int{i,1}{10};

                
                % create individual force polyzonotopes
                % 1. collapse grest at end of forming constraints 
                % where <0 so can always add and it doesn't affect 
                % calculations.
                % 2. the reduce operation that is performed in the
                % poly_zonotope_rnea() call means that the PZs output
                % might not have G (and therefore expMat and id) or
                % Grest so that is why there is if statements below
                % handling empty components of the output PZs.
                
                % centers
                Fx_poly_c = contact_poly.c(1);
                Fy_poly_c = contact_poly.c(2);
                Fz_poly_c = contact_poly.c(3);
                % generators
                if isempty(contact_poly.G)
                    Fx_poly_G = [];
                    Fy_poly_G = [];
                    Fz_poly_G = [];
                else
                    Fx_poly_G = contact_poly.G(1,:);
                    Fy_poly_G = contact_poly.G(2,:);
                    Fz_poly_G = contact_poly.G(3,:);
                end
                % Grest generators
                if isempty(contact_poly.Grest)
                    Fx_poly_Grest = [];
                    Fy_poly_Grest = [];
                    Fz_poly_Grest = [];
                else
                    Fx_poly_Grest = contact_poly.Grest(1,:);
                    Fy_poly_Grest = contact_poly.Grest(2,:);
                    Fz_poly_Grest = contact_poly.Grest(3,:);
                end
                % exponent matrices
                if isempty(contact_poly.expMat)
                    Fx_poly_expMat = [];
                    Fy_poly_expMat = [];
                    Fz_poly_expMat = [];
                else
                    Fx_poly_expMat = contact_poly.expMat;
                    Fy_poly_expMat = contact_poly.expMat;
                    Fz_poly_expMat = contact_poly.expMat;
                end
                % id matrix
                if isempty(contact_poly.id)
                    Fx_poly_id = [];
                    Fy_poly_id = [];
                    Fz_poly_id = [];
                else
                    Fx_poly_id = contact_poly.id;
                    Fy_poly_id = contact_poly.id;
                    Fz_poly_id = contact_poly.id;
                end
                % creating individual force polyzonotopes
                Fx_poly = armour.pz_roahm.polyZonotope_ROAHM(Fx_poly_c,Fx_poly_G,Fx_poly_Grest,Fx_poly_expMat,Fx_poly_id);
                Fy_poly = armour.pz_roahm.polyZonotope_ROAHM(Fy_poly_c,Fy_poly_G,Fy_poly_Grest,Fy_poly_expMat,Fy_poly_id);
                Fz_poly = armour.pz_roahm.polyZonotope_ROAHM(Fz_poly_c,Fz_poly_G,Fz_poly_Grest,Fz_poly_expMat,Fz_poly_id);

                % which way is the Fz/normal returned by rnea
                % pointing? upwards out of the tray

                % Need to create the correct moment vectors for the ZMP
                % calculation. The moments need to have the
                % cross(distance,force) added to them.
%                     ZMP_Moment{i,1} = n_int{i,1}{10} + cross([0;0;cup_height],f_int{i,1}{10});

                % separation constraint: Fnormal < 0
                % ? does this need to be multiplied by -1? yes 
                % looking at the testing in
                % Waiter_urdf_RNEA_Testing.m, the cup upright on
                % the tray has RNEA output a positive normal force
                % in the z-axis. This means that Fz>0 is the
                % constraint so to rewrite as -1.*Fz<0 it
                % needs to be multiplied by a -1.
                sep_poly_temp = -1.*Fz_poly; % verified (matches regular rnea and constraint, but slightly more negative (safer) than regular value. should subtract Grest instead?)
                sep_poly{i,1} = armour.pz_roahm.polyZonotope_ROAHM(sep_poly_temp.c + sum(abs(sep_poly_temp.Grest)),sep_poly_temp.G,[],sep_poly_temp.expMat,sep_poly_temp.id);
                % create new pz with grest collapsed

                % slipping constraint: Ftanx^2+Ftany^2 < u_s^2*Fnorm^2
                % this is rewritten as:
                % Ftanx^2+Ftany^2 - u_s^2*Fnorm^2 < 0

                slip_poly_temp = Fx_poly.*Fx_poly + Fy_poly.*Fy_poly - self.u_s^2*Fz_poly.*Fz_poly;
                slip_poly{i,1} = armour.pz_roahm.polyZonotope_ROAHM(slip_poly_temp.c + sum(abs(slip_poly_temp.Grest)),slip_poly_temp.G,[],slip_poly_temp.expMat,slip_poly_temp.id);
                % create new pz with grest collapsed
                
                % tipping constraint version 1
%                     ZMP_top = cross(n_int{i,1}{10},[0;0;1]); % verified (same center as normal rnea)
                ZMP_top = cross([0;0;1],n_int{i,1}{10});
                % for the bottom component: 
                ZMP_bottom = f_int{i,1}{10}*[0,0,1]; % verified (same center as normal rnea)
                ZMP_temp = (ZMP_bottom.*ZMP_bottom).*(self.surf_rad)^2;
                
                % there should either be only G, only Grest or both.
                % this should handle those three cases.
                if isempty(ZMP_top.G)
                    ZMP_topx = armour.pz_roahm.polyZonotope_ROAHM(ZMP_top.c(1),[],ZMP_top.Grest(1,:),[],[]);
                    ZMP_topy = armour.pz_roahm.polyZonotope_ROAHM(ZMP_top.c(2),[],ZMP_top.Grest(2,:),[],[]);
                elseif isempty(ZMP_top.Grest)
                    ZMP_topx = armour.pz_roahm.polyZonotope_ROAHM(ZMP_top.c(1),ZMP_top.G(1,:),[],ZMP_top.expMat,ZMP_top.id);
                    ZMP_topy = armour.pz_roahm.polyZonotope_ROAHM(ZMP_top.c(2),ZMP_top.G(2,:),[],ZMP_top.expMat,ZMP_top.id);
                else
                    ZMP_topx = armour.pz_roahm.polyZonotope_ROAHM(ZMP_top.c(1),ZMP_top.G(1,:),ZMP_top.Grest(1,:),ZMP_top.expMat,ZMP_top.id);
                    ZMP_topy = armour.pz_roahm.polyZonotope_ROAHM(ZMP_top.c(2),ZMP_top.G(2,:),ZMP_top.Grest(2,:),ZMP_top.expMat,ZMP_top.id);
                end

                tip_poly_full = ZMP_topx.*ZMP_topx + ZMP_topy.*ZMP_topy - ZMP_temp;
                tip_poly{i,1} = armour.pz_roahm.polyZonotope_ROAHM(tip_poly_full.c + sum(abs(tip_poly_full.Grest)),tip_poly_full.G,[],tip_poly_full.expMat,tip_poly_full.id);

                % remove dependence of grasp constraints
                sep_poly{i,1} = remove_dependence(sep_poly{i,1},jrsInstance.k_id(end));
                tip_poly{i,1} = remove_dependence(tip_poly{i,1},jrsInstance.k_id(end));
                slip_poly{i,1} = remove_dependence(slip_poly{i,1},jrsInstance.k_id(end));
            end
            
            reachableSet = waitr.reachsets.EEInstance(sep_poly, slip_poly, tip_poly, jrsInstance);
        end
    end
end