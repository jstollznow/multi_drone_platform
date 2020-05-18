
Rate = 100;


Api = mdp_api(Rate, 'matlab_graphing');

Drones = Api.getalldrones();

DroneGraphing = mdp_drone_graphing.empty(0, length(Drones));

for i = 1 : length(Drones)
    DroneGraphing(i) = mdp_drone_graphing(Drones(i));
end
AllLanded = false;
ShutdownSeq = rosparam('get','mdp/should_shut_down');

SessionParam = '/mdp/session_directory';
Ptree = rosparam();
SessionPath = '';
if has(Ptree,SessionParam) 
    SessionPath = rosparam('get',SessionParam);
end
disp("Ready to record drone positions...");
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
legend('Location', 'northeastoutside');
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
legend('Location', 'northeastoutside');
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
legend('Location', 'northeastoutside');
hold off;
saveas(gcf,strcat(SessionPath,'XYZGraphs.png'));
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
    if length(xArr) > 1 && length(yArr) > 1
        startName = strcat(lineName, " Start");
        endName = strcat(lineName, " End");
        startPlot = plot(xArr(1), yArr(1),'linestyle','none','marker','^',...
            'MarkerSize', 8, 'DisplayName', startName);
        startPlot.MarkerFaceColor = startPlot.Color;
        hold on;
        endPlot = plot(xArr(length(xArr)), yArr(length(yArr)),'linestyle','none','marker','v',...
            'MarkerSize', 8, 'DisplayName', endName);
        endPlot.MarkerFaceColor = endPlot.Color;
        hold on;
    end
end
title('XY position');
xlabel('Position (m)');
ylabel('Position (m)');
legend('Location', 'northeastoutside');
hold off;
saveas(gcf,strcat(SessionPath, 'XYGraph.png'));
uiwait(helpdlg('Examine the figures, then click OK to finish. Figures have been exported into sessions directory.'));


