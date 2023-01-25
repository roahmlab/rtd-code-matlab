function zonotopes = zonotope_sensor(world, agent, time)
    arguments
        world struct
        agent rtd.sim.world.WorldEntity
        time = []
    end
    % flatten all objects in the world
    world = struct2array(world);
    % remove the agent, and mask only components with representations
    agent_mask = ~strcmp({world.uuid}, agent.uuid);
    rep_mask = arrayfun(@(x)isprop(x,'representation'),world);
    mask = agent_mask & rep_mask;
    % Get all the obstacles
    representations = [world(mask).representation];
    zonotopes = arrayfun(@(x)x.get_zonotope(time=time),representations);
end
