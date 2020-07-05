
Rate = 100;

Api = mdp_api(Rate, 'matlab_graphing');

ShutdownSeq = rosparam('get','mdp/should_shut_down');
if ShutdownSeq
  return 
end
Drones = Api.getalldrones();

ShutdownSeq = rosparam('get','mdp/should_shut_down');
if isempty(Drones) || ShutdownSeq
   return 
end

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
subplot(3,3,1);
for i = 1 : length(DroneGraphing)
    if ~isempty(DroneGraphing(i).get_X())
        lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
        graph = plot(DroneGraphing(i).get_Time(), DroneGraphing(i).get_X(),...
            'DisplayName', lineName);
        DroneGraphing(i) = DroneGraphing(i).set_Color(graph.Color);
        hold on;
    end
end
title('X position');
xlabel('Time (Seconds)');
ylabel('Position (m)');
% legend('Location', 'northeastoutside');
hold off;
%Y Coord
subplot(3,3,4);
for i = 1 : length(DroneGraphing)
    if ~isempty(DroneGraphing(i).get_Y())
        lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
        graph = plot(DroneGraphing(i).get_Time(), DroneGraphing(i).get_Y(),...
            'DisplayName', lineName);
        graph.MarkerFaceColor = DroneGraphing(i).get_Color();
        graph.Color = DroneGraphing(i).get_Color();
        hold on;
    end
end
title('Y position');
xlabel('Time (Seconds)');
ylabel('Position (m)');
% legend('Location', 'northeastoutside');
hold off;
%Z Coord
subplot(3,3,7);

for i = 1 : length(DroneGraphing)
    if ~isempty(DroneGraphing(i).get_Z())
        lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
        graph = plot(DroneGraphing(i).get_Time(), DroneGraphing(i).get_Z(),...
            'DisplayName', lineName);
        graph.MarkerFaceColor = DroneGraphing(i).get_Color();
        graph.Color = DroneGraphing(i).get_Color();
        hold on;
    end
end
title('Z position');
xlabel('Time (Seconds)');
ylabel('Position (m)');
% legend('Location', 'northeastoutside');
hold off;
subplot(3, 3, [2 5 8]);
% movegui('northeast');
% saveas(gcf,strcat(SessionPath,'XYZGraphs.png'));
% figure(2);
% clf;
% lgd = legend;
% lgd.NumColumns = idivide(0, length(DroneGraphing), 'ceil');
for i = 1 : length(DroneGraphing)
    if ~isempty(DroneGraphing(i).get_Time())
        lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
        xArr = DroneGraphing(i).get_X();
        yArr = DroneGraphing(i).get_Y();
        graph = plot(xArr, yArr, 'DisplayName', lineName);
        graph.MarkerFaceColor = DroneGraphing(i).get_Color();
        graph.Color = DroneGraphing(i).get_Color();
        hold on;    
        if length(xArr) > 1 && length(yArr) > 1
            startName = strcat(lineName, " Start");
            endName = strcat(lineName, " End");
            startPlot = plot(xArr(1), yArr(1),'linestyle','none','marker','^',...
                'MarkerSize', 6, 'DisplayName', startName);
            startPlot.MarkerFaceColor = DroneGraphing(i).get_Color();
            startPlot.Color = DroneGraphing(i).get_Color();
            startPlot.Annotation.LegendInformation.IconDisplayStyle = 'off';
            hold on;
            endPlot = plot(xArr(length(xArr)), yArr(length(yArr)),'linestyle','none','marker','v',...
                'MarkerSize', 6, 'DisplayName', endName);
            endPlot.MarkerFaceColor = DroneGraphing(i).get_Color();
            endPlot.Color = DroneGraphing(i).get_Color();
            endPlot.Annotation.LegendInformation.IconDisplayStyle = 'off';
            hold on;
        end
    end
end
% added to show only one set of start and end markers
plot(NaN, NaN, 'linestyle', 'none', 'marker', '^',...
    'MarkerSize', 6, 'DisplayName', "Start", ...
    'MarkerFaceColor', 'black', 'MarkerEdgeColor', 'black');
hold on;
plot(NaN, NaN, 'linestyle', 'none', 'marker', 'v',...
    'MarkerSize', 6, 'DisplayName', "End", ...
    'MarkerFaceColor', 'black', 'MarkerEdgeColor', 'black');
hold on;
title('XY position');
xlabel('X Pos (m)');
ylabel('Y Pos (m)');
legend('Location', 'northoutside');
hold off;

subplot(3,3,[3 6 9]);
for i = 1 : length(DroneGraphing)
    if ~isempty(DroneGraphing(i).get_Time())
        lineName = strcat("drone ", num2str(DroneGraphing(i).get_ID().NumericId));
        graph = plot(DroneGraphing(i).get_Time(), DroneGraphing(i).get_Dist(), 'DisplayName', lineName);
        graph.MarkerFaceColor = DroneGraphing(i).get_Color();
        graph.Color = DroneGraphing(i).get_Color();
        hold on;    
    end
end
title('Distance to nearest obstacle');
xlabel('Time (seconds)');
ylabel('Distance (m)');
hold off;

set(gcf, 'units', 'normalized', 'position', [0.1 0.5 0.8 0.5]);
saveas(gcf, strcat(SessionPath, 'AllPlots.png'));
saveas(gcf, strcat(SessionPath, 'AllPlots.fig'));
save(strcat(SessionPath, '/data.mat'));
uiwait(helpdlg('Examine the figures, then click OK to finish. Figures have been exported into sessions directory.'));
movegui('south');


