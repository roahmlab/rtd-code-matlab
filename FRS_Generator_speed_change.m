%constraints and parameters
%write functions for load mega and load u0 intervals

classdef FRS_Generator_speed_change < rtd.planner.reachsets.ReachSetGenerator

   %load FRS data for speed change
   %load dummy variables for testing
       
    %properties contains the z property and meta data
    properties
        
        robot %car
        frsGenerator %FRS generator similar to jrsGenerator
        obs_frs_combs %A structure parameter with two fields: "maxcombs" defining the maximum number of combinations and "combs" storing the generated combinations.
        speed_lat %lateral speed v(t)
        speed_log %logitudnal speed u(t)
        state %consists of x & y x(t) & y(t)
        head %heading of the vehicle h(t)
        yaw_rate %r rad/sec
        u0 %initial logitudnal speed
        v0 %initial lateral speed
        r0 %initial yaw rate
        p_u %slice parameter for speed
        t %time
        brakeidx1
        brakeidx2
        vehrs
        %p_y %slice parameter for lane or direction change
        
                
    end
     
    %FRS speed generator constructor
    methods
        function self = FRS_Generator_speed_change(robot, frsGenerator,z_matrix ,options)%pass all properties
            arguments
                
                robot armour.ArmourAgent
                frsGenerator rtd.reachsets.ReadchSetGenerator
                z_matrix {mustBeFile} = 'Offline_Reachability_Analysis/FRSdata/ONLY_Z.m'%load
                options.obs_frs_combs(1,1) struct = struct('maxcombs', 200, 'combs', [])
                options.verboseLevel(1,1) rtd.util.types.LogLevel = "DEBUG"
                
            end
            self.robot = robot;
            self.frsGenerator = frsGenerator;
            self.obs_frs_combs = options.obs_frs_combs;
            self.speed_log = z_matrix.speed_log;
            self.speed_lat = z_matrix.speed_lat;
            self.state.x = z_matrix.x;
            self.state.y = z_matrix.y;
            self.head = z_matrix.h;
            self.yaw_rate = z_matrix.r;
            self.u0 = z_matrix.u0;
            self.v0 = z_matrix.r0;
            self.r0 = z_matrix.r0;
            self.p_u = z_matrix.p_u;
            self.t = z_matrix.t;
            self.vehrs = vehrs; %from file
            self.brakeidx1 = brakeidx1;
            self.brakeidx2 = brakeidx2;

            if isempty(self.obs_frs_combs.combs)
                self.obs_frs_combs.combs = ...
                    generate_combinations_upto(self.obs_frs_combs.maxcombs);
            end
            self.set_vdisplevel(options.verboseLevel);
        end
    end

    methods (Access=protected)
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns FRS Instance
        function reachableSet = generateReachableSet(self, robotState)
           
           
            frsInstance = self.frsGenerator.getReachableSet(robotState, ignore_cache=false); %right can be used for any robot state
            frstotal = FrsTotal(load_file);
            %define slc_vals = ...
            zonosliceinfo = ZonoSliceInfo(slc_vals,self.u0,self.v0,self.r0);
            reachableSet = refine.Offline_Reachability_Analysis.FRS_Instance_speed_change(self.p_u,zonosliceinfo,self.vehrs,frstotal.mega,frstotal,self.brakeidx1,self.brakeidx2);
        end
    end


end

%% function for FRSTotal

function frs_total = FrsTotal(f_name)
    disp(['Loading FRS ',f_name]);
    frs_total = struct();
    frs_total.successful = true;
    num_mega = int32(0);
    file = fopen('f_name.txt','r');%name of the f_name files
    data = textscan(file,'%s %f %f');%cell array
    % fclose('f_name.txt');
    in_str = data{1}; 
    frs_total.u0_min = data{2};
    frs_total.u0_max = data{3};

    %u0 min and max
    if ~strcmp(in_str,"MINMAX_U0")
        disp(' in_str ~= "MINMAX_U0');
        frs_total.successful = false;
        return;
    end

    %alternating au
    data = textscan(file,'%s %f');
    in_str = data{1};
    frs_total.alternating_au = data{2};
    if ~strcmp(in_str,"ALTERNATING_AU")
        disp(' in_str ~= "ALTERNATING_AU" ');
        frs_total.successful = false;
        return;
    end

    %set u0
    data = textscan(file,'%s %f');
    in_str = data{1};
    frs_total.num_megas = data{2};
    if ~strcmp(in_str,"NUM_MEGAS")
       disp(' num_megas ~= "NUM_MEGAS"');
       frs_total.successful = false;
       return;
    end


    %num_megas
    if (frs_total.num_megas <=0)
       disp(' num_megas <= 0');
       frs_total.successful = false;
       return;
    end
    
    %frs mega
    frs_total.mega.reshape(frs_total.num_megas);
    for i = 1:frs_total.num_megas+1
        success = LoadMega(file, frs_total.mega(i));
        if ~success
            disp('LoadMega failed');
            frs_total.successful = false;
            print('Failure ',f_name);
            return;
        end
    end

    %LoadU0 Interval
    success = Loadu0interval(file, frs_total.u0_intervals_);
    if ~success
        disp('u0 intervals failed');
        frs_total.successful = false;
        return;
    end
    
    %close the file
    fclose('f_name.txt');%replace it with the name of the file

    %u0 interval size
    if(size(frs_total.u0_intervals_) ~= size(frs_total.mega))
        disp('u0_interval size mismatch');
        frs_total.successful = false;
        return;
    end

    %if successful = true
    if(frs_total.successful)
        print('Succesfully loaded',f_name);
        print('FRS MEGAS',size(frs_total.mega));
    else
        print('FAILURE');
        return;
    end

end

%% function for ZonoSliceInfo

function ret = ZonoSliceInfo(slc_vals_,u,v,r)%pass slc_vals_
    ret = struct();
    ret.lambda_ok = true;
    ret.slc_xy_set = false;
    for i = 1:numel(slc_vals_)
        sliceable = slc_vals_(i);
        sl_dim = sliceable.dim;
        sl_val = sliceable.slc_vals_;

        if(sl_dim == 11 || sl_dim == 12)
            ret.slc_x_ = sliceable.x_;
            ret.slc_y_ = sliceable.y_;
            ret.slc_h_ = sliceable.h_;
            ret.slc_val_ = sl_val;
            ret.center_slc_val_ = sliceable.center_val_;
            ret.slc_xy_set_ = true;
        else
            slice_pt = 0.0;
            if sl_dim == 7
                slice_pt = u;
            elseif sl_dim == 8
                slice_pt = v;
            elseif sl_dim == 9
                slice_pt = r;
            else
                error('Invalid sl_dim value.');
            end
        end
        kLambdaEps = 1.0e-9;
        slice_lambda = (slice_pt - sliceable.center_val_) / sl_val;
        abs_lambda = abs(slice_lambda);
        if (abs_lambda > 1.0 && abs_lambda <= 1.0 + kLambdaEps)
            if (slice_lambda < 0)
              slice_lambda = -1.0;
            else
              slice_lambda = 1.0;
           end
        end
        abs_lambda = abs(slice_lambda);
        if (abs_lambda > 1.0)
            disp("Slice point out of bounds");
                disp("slice_pt:              " + slice_pt);
                disp("sliceable.center_val_: " + sliceable.center_val_);
                disp("sl_dim:                " + sl_dim);
                disp("sl_val:                " + sl_val);
                disp("sl_lambda:             " + slice_lambda);

        end
        ret.lambda_ok_ = abs(slice_lambda) <= 1.0;
        ret.x_sliced_sum_ = ret.x_sliced_sum_ + slice_lambda * sliceable.x_;
        ret.y_sliced_sum_ = ret.y_sliced_sum_ + slice_lambda * sliceable.y_;
        ret.h_sliced_sum_ = ret.h_sliced_sum_ + slice_lambda * sliceable.h_;

    end
end

%% Load u0 interval function

function success = Loadu0interval(f_in, intervals)
    file = fopen(f_in);
    data = textscan(file, '%s %d %d');
    in_str = data{1}{1};
    u0_rows = data{2};
    u0_cols = data{3};
    if ~strcmp(in_str, 'u0_intervals')
        disp('in_str ~= "u0_intervals"');
        success = false;
        fclose(file);
        return;
    end
    if u0_rows ~= 2
        disp('u0_rows ~= 2');
        success = false;
        fclose(file);
        return;
    end
    intervals = repmat(struct('Min', 0, 'Max', 0), 1, u0_cols);
    for i = 1:u0_cols
        tmp_val_min = fscanf(file, '%f', 1);
        intervals(i).Min = tmp_val_min;
    end
    for i = 1:u0_cols
        tmp_val_max = fscanf(file, '%f', 1);
        intervals(i).Max = tmp_val_max;
    end
    success = true;
    fclose(file);
end


%% Load Mega function

function success = LoadMega(f_in, mega)
    file = fopen(f_in);
    in_str = fscanf(file, '%s', 1);
    if ~strcmp(in_str, 'M_MEGA')
        disp('in_str ~= "M_MEGA"');
        success = false;
        fclose(file);
        return;
    end
    in_str = fscanf(file, '%s', 1);  % mega idx
    in_str = fscanf(file, '%s', 1);  % (MEGA_{N})

    % Load AU
    manu_type = fscanf(file, '%s', 1);
    au_num = fscanf(file, '%d', 1);
    if ~strcmp(manu_type, 'AU')
        disp('manu_type ~= "AU"');
        success = false;
        fclose(file);
        return;
    end
    mega.au_ = repmat(struct(), 1, au_num);
    for i = 1:au_num
        if ~LoadVehrs(file, mega.au_(i))
            disp('AU LoadVehrs failed');
            success = false;
            fclose(file);
            return;
        end
    end

    success = true;
    fclose(file);
end






