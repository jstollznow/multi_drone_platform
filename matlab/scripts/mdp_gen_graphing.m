
Rate = 100;


Api = mdp_api(Rate, 'matlab_graphing');

Drones = Api.getalldrones();

DroneGraphing = mdp_drone_graphing.empty(0, length(Drones));

for i = 1 : length(Drones)
    DroneGraphing(i) = mdp_drone_graphing(Drones(i));
end
AllLanded = false;
ShutdownSeq = rosparam('get','mdp/should_shut_down');

while (~AllLanded || ~ShutdownSeq) 
    AllLanded = true;
    if isempty(Drones)
       break; 
    end
    for i = 1 : length(Drones)
        ShutdownSeq = rosparam('get','mdp/should_shut_down');            
        State = Api.getstate(Drones(i));
        if State == mdp_flight_state.HOVERING || State == mdp_flight_state.MOVING
            AllLanded = false;
            DroneGraphing(i) = DroneGraphing(i).add_data(Api.getposition(Drones(i)));
        end
    end
end

delete(Api);

% plot the data
figure(1);
clf;
%X Coord
subplot(3,1,1);
for i = 1 : length(DroneGraphing)
    lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
    plot(DroneGraphing(i).get_Time(), DroneGraphing(i).get_X(),...
        'DisplayName', lineName);
    hold on;
end
title('X position');
xlabel('Time (Seconds)');
ylabel('Position (m)');
legend();
hold off;
%Y Coord
subplot(3,1,2);
for i = 1 : length(DroneGraphing)
    lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
    plot(DroneGraphing(i).get_Time(), DroneGraphing(i).get_Y(),...
        'DisplayName', lineName);
    hold on;
end
title('Y position');
xlabel('Time (Seconds)');
ylabel('Position (m)');
legend();
hold off;
%Z Coord
subplot(3,1,3);

for i = 1 : length(DroneGraphing)
    lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
    plot(DroneGraphing(i).get_Time(), DroneGraphing(i).get_Z(),...
        'DisplayName', lineName);
    hold on;
end
title('Z position');
xlabel('Time (Seconds)');
ylabel('Position (m)');
legend();
hold off;

figure(2);
clf;
% lgd = legend;
% lgd.NumColumns = idivide(0, length(DroneGraphing), 'ceil');
for i = 1 : length(DroneGraphing)
    lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
    xArr = DroneGraphing(i).get_X();
    yArr = DroneGraphing(i).get_Y();
    plot(xArr, yArr, 'DisplayName', lineName);
    hold on;
    startEndX = [xArr(1) xArr(length(xArr)-1)];
    startEndY = [yArr(1) yArr(length(yArr)-1)];
    startEndName = strcat(lineName, ' (Start-End)');
    plot(startEndX, startEndY,'linestyle','none','marker','*',...
        'MarkerSize',12 , 'DisplayName', startEndName);
    hold on;
end
title('XY position');
xlabel('Position (m)');
ylabel('Position (m)');
legend();
hold off


