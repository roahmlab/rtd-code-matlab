classdef testclass < OptionsClass & handle
    methods (Static)
        function options = defaultoptions()
            options.test = 1
        end
    end
end
