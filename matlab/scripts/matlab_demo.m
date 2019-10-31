Api = mdp_api(10);

Drones = Api.getalldrones();
if length(Drones) >= 1
    
    Api.cmdtakeoff(Drones(1), 0.5, 2.0);
    % Api.cmdtakeoff(Drones(2), 0.5, 2.0);

    Api.sleepuntilidle(Drones(1));
    % Api.sleepuntilidle(Drones(2));

    Pos = mdp_position_msg;
    Pos.Relative = true;
    Pos.KeepHeight = true;
    Pos.Position = [0.5  0.0  1.0];
    Pos.Duration = 4.0;
    Pos.Yaw = 0.0;

    Api.setposition(Drones(1), Pos);
    % Api.setposition(Drones(2), Pos);

    Api.sleepuntilidle(Drones(1));
    % Api.sleepuntilidle(Drones(2));

    Api.cmdgohome(Drones(1), 0.5);
    % Api.cmdgohome(Drones(2), 0.5);

    Api.sleepuntilidle(Drones(1));
    % Api.sleepuntilidle(Drones(2));

    Api.cmdland(Drones(1));
    % Api.cmdland(Drones(2));

    Api.sleepuntilidle(Drones(1));
    % Api.sleepuntilidle(Drones(2));

end

delete(Api);