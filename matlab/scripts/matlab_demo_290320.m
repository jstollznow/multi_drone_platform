
Rate = 100;


Api = mdp_api(Rate, 'matlab_demo');

Drones = Api.getalldrones();

% takeoff drones
Api.cmdtakeoff(Drones(1), 0.5, 4.0);
Api.cmdtakeoff(Drones(2), 1.0, 4.0);

% create pos message
PosMsg = mdp_position_msg;
PosMsg.KeepHeight = true;
PosMsg.Duration = 2.0;

WayPos = Api.getposition(Drones(1));

FolMsg = mdp_position_msg;
FolMsg.Position(1) = WayPos.X;
FolMsg.Position(2) = WayPos.Y;
FolMsg.Position(3) = 0.0;
FolMsg.KeepHeight = true;
FolMsg.Duration = 2.0;

% define waypoints
Positions = [
    [-0.5, -0.5, 0.0];
    [-0.5, 0.5, 0.0];
    [0.5, 0.5, 0.0];
    [0.5, -0.5, 0.0]
];

% define plotting arrays
WayDronePosX = [];
FolDronePosX = [];
WayDronePosY = [];
FolDronePosY = [];

% wait till drones geet to their heights
Api.sleepuntilidle(Drones(1));
Api.sleepuntilidle(Drones(2));

% put follow drone on top of pos drone
Api.setposition(Drones(2), FolMsg);
Api.sleepuntilidle(Drones(2));
FolMsg.Duration = 0.5;

% go throught the waypoints
for n = 1 : length(Positions)
    % set drone 1's position to new waypoint
    PosMsg.Position = Positions(n, :);
    Api.setposition(Drones(1), PosMsg);
    
    % expect moving (as it is possible that the drone server has not yet
    % updated the drones state from HOVERING)
    State = mdp_flight_state.MOVING;
    
    % while waiting for drone 1 to reach the waypoint...
    while State ~= mdp_flight_state.HOVERING
        WayPos = Api.getposition(Drones(1));
        FolMsg.Position(1) = WayPos.X;
        FolMsg.Position(2) = WayPos.Y;
        Api.setposition(Drones(2), FolMsg);
        
        % plot the drone positions
        FolPos = Api.getposition(Drones(2));
        WayDronePosX = [WayDronePosX WayPos.X];
        WayDronePosY = [WayDronePosY WayPos.Y];
        FolDronePosX = [FolDronePosX FolPos.X];
        FolDronePosY = [FolDronePosY FolPos.Y];
        
        State = Api.getstate(Drones(1));
    end
end

% tell drones to go home
Api.cmdgohome(Drones(1), 0.0, 5.0);
Api.cmdgohome(Drones(2), 0.0, 5.0);

% wait until they arive
Api.sleepuntilidle(Drones(1));
Api.sleepuntilidle(Drones(2));

% shutdown api
delete(Api);

% plot the data
clf
Iters = (1:length(WayDronePosX));
subplot(2,1,1);
plot(Iters, WayDronePosX);
hold on
plot(Iters, FolDronePosX);
title('X position');
legend('Waypoint Drone', 'Follow Drone');
axis([0 length(WayDronePosX) -1 1]);
subplot(2,1,2);
plot(Iters, WayDronePosY);
hold on
plot(Iters, FolDronePosY);
title('Y position');
legend('Waypoint Drone', 'Follow Drone');
axis([0 length(WayDronePosX) -1 1]);
hold off