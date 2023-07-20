classdef MexWrapper < handle
% Autocompile mex wrapper for mex classes and functions
%
% Utility wrapper
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-02-08
% Last Revised: 2023-03-18
%
% See also mex
%
% --- More Info ---
%

    methods (Static, Access=protected, Abstract)
        fcns = mex_functionsToBuild()
        [src_folder, common_files, compile_options] = mex_commonCompileInfo()
        out = mex_checkWhich(names)
    end

    methods
        function self = MexWrapper()
            % If any mex functions we want don't exist, compile
            if self.checkMexNotExists()
                fprintf("%s mex functions don't exist, building!\n", class(self))
                self.compile()
            end

            % Check if we need to do any rebuilds
            try
                if self.checkFileDateTimes()
                    fprintf("%s mex source folder updated, building!\n", class(self))
                    self.compile()
                end
            catch
                % If we're here, it also means they don't exist
                fprintf("%s has unexpected mex state, building!\n", class(self))
                self.compile()
            end
        end
    end

    methods (Access=private)
        % Verify the mex files exist
        function mex_not_exists = checkMexNotExists(self)
            % Get the list of functions
            fcns = self.mex_functionsToBuild();
            fcn_list = string(fieldnames(fcns).');

            % Find them
            fcn_list_locations = arrayfun(@self.mex_checkWhich, fcn_list, UniformOutput=false);
            mex_not_exists = any(cellfun(@isempty, fcn_list_locations));

            % Verify that these files are actually in the right place
            if ~mex_not_exists
                classpath = basepath(self);
                valid_path = cellfun(@(path) startsWith(path, classpath, IgnoreCase=true), fcn_list_locations);
                if ~all(valid_path)
                    warning("Expected mex functions are being masked by another function elsewhere! Treating files as nonexisting!")
                    mex_not_exists = true;
                end
            end
        end

        % Check if we need to do a compilation update
        function files_updated = checkFileDateTimes(self)
            % Get the list of functions
            fcns = self.mex_functionsToBuild();
            fcn_list = string(fieldnames(fcns).');

            % Get the earliest mex'd time
            fcn_list_locations = arrayfun(@self.mex_checkWhich, fcn_list, UniformOutput=false);
            fcn_filedate = cellfun(@dir,fcn_list_locations);
            fcn_filedate = min([fcn_filedate.datenum]);

            % Get the latest source file changes incl. subdirectories
            [src_folder, ~, ~] = self.mex_commonCompileInfo();
            srcfilepath = fullfile(basepath(self), src_folder, '**');
            latest_filedate = dir(srcfilepath);
            latest_filedate = latest_filedate(~[latest_filedate.isdir]);
            latest_filedate = max([latest_filedate.datenum]);

            % True if the latest source is later than the earliest mex time
            files_updated = latest_filedate > fcn_filedate;
        end

        % Do a compilation run for all files in this class.
        function compile(self)
            [src_folder, common_files, options] = self.mex_commonCompileInfo();
            % Extra compilation options as string array
            comp_options = string(options);
            comp_options = comp_options(:).';

            % Get the base path
            currpath = basepath(self);
            mexfilepath = fullfile(currpath, 'private');
            objfilepath = fullfile(currpath, 'private', 'obj');
            srcfilepath = fullfile(currpath, src_folder);

            % Compilation lists
            core_list = string(common_files);
            core_list = core_list(:).';

            % Get the list of functions
            fcns = self.mex_functionsToBuild();
            fcn_list = string(fieldnames(fcns).');

            % First build the common files
            if ~isempty(core_list)
                fprintf("Building common files!\n");
                abs_mask = arrayfun(@(path) startsWith(path, '/'), core_list);
                resolved_files = fullfile(srcfilepath, core_list(~abs_mask));
                core_files = [core_list(abs_mask), resolved_files];
                mex_command = ["-outdir", objfilepath, "-c", core_files, comp_options];
                mex_command = num2cell(mex_command);
                mex(mex_command{:});
            end

            % Update the common file outputs
            [~, core_list_names, ~] = fileparts(core_list);
            core_files_precompiled = fullfile(objfilepath, core_list_names + ".o");

            % Build compilation commands
            for fcn=fcn_list
                fprintf("Building %s!\n", fcn);
                
                % Retrieve data (or struct data if struct)
                data = fcns.(fcn);
                fcn_srcs = data;
                fcn_opts = [];
                if isfield(data, 'srcs')
                    fcn_srcs = data.srcs;
                end
                if isfield(data, 'opts')
                    fcn_opts = data.opts;
                end

                % Resolve the filenames
                fcn_files = string(fcn_srcs);
                input_files = fcn_files(:).';

                % seperate the absolute files and resolve the relative ones
                abs_mask = arrayfun(@(path) startsWith(path, '/'), input_files);
                resolved_files = fullfile(srcfilepath, input_files(~abs_mask));
                all_files = [input_files(abs_mask), resolved_files, core_files_precompiled];

                % Generate the mex command and run it
                mex_command = ["-outdir", mexfilepath, "-output", fcn, all_files, comp_options, fcn_opts];
                mex_command = num2cell(mex_command);
                mex(mex_command{:});
            end

            % Delete common files
            if isfolder(objfilepath)
                priorState = recycle('off');
                rmdir(objfilepath, 's');
                recycle(priorState);
            end
        end
    end
end

% Utility to retrieve the basepath of the current class file.
function currpath = basepath(obj)
    currfile = which(class(obj));
    [currpath, ~, ~] = fileparts(currfile);
end