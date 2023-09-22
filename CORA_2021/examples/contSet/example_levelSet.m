function completed = example_levelSet()
% example_levelSet - example instantiation of levelSet objects
%
% Syntax:  
%    completed = example_levelSet()
%
% Inputs:
%    -
%
% Outputs:
%    completed - boolean
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: none
%
% Author:        ---
% Written:       ---
% Last update:   ---
% Last revision: ---

%------------- BEGIN CODE --------------

% construct level sets
syms x y
eq = sin(x) + y;

ls1 = levelSet(eq,[x;y],'==');
ls2 = levelSet(eq,[x;y],'<=');

% visualize the level sets
subplot(1,2,1)
xlim([-1.5,1.5]);
ylim([-1,1]);
plot(ls1,[1,2],'b');

subplot(1,2,2)
xlim([-1.5,1.5]);
ylim([-1,1]);
plot(ls2,[1,2],'r','FaceAlpha',0.5);

% example completed
completed = 1;

%------------- END OF CODE --------------