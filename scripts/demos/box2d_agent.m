%% reset

clear ; clc; close all;

%% create agent

agentinfo = demos.box2d.BoxAgentInfo(width=2, height=1, color=[170/255 107/255 220/255]);
agentstate = rtd.entity.components.GenericEntityState(agentinfo, n_states=2); % give it two states (x_pos, y_pos)
agentvisual = demos.box2d.BoxAgentVisual(agentinfo, agentstate, face_opacity=0.8);

boxagent = demos.box2d.BoxAgent(info=agentinfo, state=agentstate, visual=agentvisual);
%disp(boxagent);

%% add states (i.e., move agent around)

% default position at time=0 is (0, 0)
boxagent.state.commit_state_data(2, [0; -2]);   % (0, -2) at t=2
boxagent.state.commit_state_data(5, [-2; 1]);   % (-2, 1) at t=7
boxagent.state.commit_state_data(2, [-4; -1]);  % (-4,-1) at t=9
boxagent.state.commit_state_data(3, [4; 3]);    % (4,  3) at t=12
boxagent.state.commit_state_data(2, [2; 0]);    % (2,  0) at t=14
for i = 0:pi/20:2*pi
    boxagent.state.commit_state_data(0.25, [3*cos(i); 3*sin(i)]);
end
boxagent.state.get_state()

%% set up visual system and animate
vs = rtd.sim.systems.patch_visual.PatchVisualSystem(...
    dynamic_objects=boxagent.visual, ...
    view=2, xlim=[-5 5], ylim=[-5 5]);
vs.updateVisual(24.25);   % aniamte the first 24.25 t