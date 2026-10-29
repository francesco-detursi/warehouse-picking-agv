function fig = showExecution(run,folder)
map=run.map;cfg=run.config;log=run.trajectory;
screen=get(groot,'ScreenSize');
fig=figure('Name','Warehouse Picking Robot | Mission replay','NumberTitle','off','Color','white', ...
    'MenuBar','none','ToolBar','none','Position',[40 60 min(1240,screen(3)-80) min(760,screen(4)-120)], ...
    'Tag','WarehouseExecution');
warehouse.view.drawLayout(fig,map,cfg,false);
labels=findall(fig,'Type','textboxshape');
for k=1:numel(labels)
    if labels(k).Position(1)>=0.75,delete(labels(k));
    elseif contains(string(labels(k).String),'Single-robot layout'),labels(k).String='Warehouse A  /  Simulated mission replay';
    elseif contains(string(labels(k).String),'Dashed boxes:'),labels(k).String='Blue: travelled path  |  Grey: original plan  |  Amber dashed: updated route';end
end
ax=findobj(fig,'Tag','WarehouseMap');
delete(findobj(ax,'Type','patch'));
lines=findobj(ax,'Type','line');
for k=1:numel(lines),if isequal(lines(k).Color,[1 1 1]),delete(lines(k));end,end
plot(ax,run.plan.path(:,1),run.plan.path(:,2),'--','Color',[0.65 0.69 0.72],'LineWidth',1);
updatedRoute=plot(ax,nan,nan,'--','Color',[0.75 0.40 0.05],'LineWidth',1.1);
trail=plot(ax,nan,nan,'Color',[0.04 0.35 0.66],'LineWidth',1.6);
body=patch(ax,nan,nan,[0.04 0.35 0.66],'EdgeColor',[0.12 0.18 0.23],'LineWidth',1.2);
nose=plot(ax,nan,nan,'w-','LineWidth',2);
rackBorders=gobjects(numel(run.plan.rackIndices),1);
for j=1:numel(rackBorders)
    rackBorders(j)=rectangle(ax,'Position',map.obstacles(run.plan.rackIndices(j),:), ...
        'EdgeColor',[0.04 0.35 0.66],'LineWidth',2);
end
warehouse.view.protectMapLabels(ax);
uistack(body,'top');uistack(nose,'top');
hasPeople=isfield(run,'scenario');
if hasPeople
 [humanBodies,humanLabels]=warehouse.view.drawPedestrians(ax,run.scenario,reshape(run.pedestrians(1,:),2,[])');
end
textBox('MISSION STATUS',0.80,12,true);
phaseLabel=textBox('',0.74,13,true);
targetLabel=textBox('',0.68,11,false);
picksLabel=textBox('',0.62,11,false);
replanLabel=textBox('',0.56,10,false);
textBox('SIMULATION DATA',0.50,12,true);
timeLabel=textBox('',0.46,11,false);
distanceLabel=textBox('',0.40,11,false);
speedLabel=textBox('',0.34,11,false);
textBox(sprintf('Loading: %.0f s / rack',run.motion.loadingTime),0.24,10,false);
textBox(sprintf('Unloading: %.0f s',run.motion.unloadingTime),0.20,10,false);
uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.765 0.115 0.10 0.038], ...
    'String','Replay','Callback',@restart);
pauseButton=uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.875 0.115 0.10 0.038], ...
    'String','Pause','Callback',@togglePause);
rate=uicontrol(fig,'Style','popupmenu','Units','normalized','Position',[0.765 0.065 0.10 0.038], ...
    'String',{'1x','5x','10x','20x'},'Value',2,'Callback',@changeRate);
uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.875 0.065 0.10 0.038], ...
    'String','Export figure','Callback',@exportFigure);
index=1;simTime=0;lastTick=tic;paused=false;rates=[1 5 10 20];
fig.CloseRequestFcn=@closeViewer;
setappdata(fig,'RunFolder',folder);
render();
while isgraphics(fig)
    if ~paused
        simTime=min(log.Time_s(end),simTime+toc(lastTick)*rates(rate.Value));
        index=find(log.Time_s<=simTime,1,'last');
        render();
        if index==height(log),paused=true;pauseButton.String='Finished';end
    end
    lastTick=tic;
    drawnow;
    pause(0.015);
end
    function h=textBox(value,y,font,bold)
        weight='normal';if bold,weight='bold';end
        h=annotation(fig,'textbox',[0.76 y 0.235 0.045],'String',value,'EdgeColor','none', ...
            'FontName','Arial','FontSize',font,'FontWeight',weight,'Color',[0.12 0.18 0.23],'Interpreter','none');
    end
    function render()
        nextIndex=min(index+1,height(log));weight=0;
        if nextIndex>index,weight=(simTime-log.Time_s(index))/(log.Time_s(nextIndex)-log.Time_s(index));end
        pose=(1-weight)*[log.X_m(index) log.Y_m(index) log.Heading_rad(index)]+ ...
            weight*[log.X_m(nextIndex) log.Y_m(nextIndex) log.Heading_rad(nextIndex)];
        p=warehouse.geometry.robotPolygon(pose,cfg);body.XData=p(:,1);body.YData=p(:,2);
        nose.XData=[pose(1) pose(1)+0.4*cos(pose(3))];nose.YData=[pose(2) pose(2)+0.4*sin(pose(3))];
        visible=unique([1:5:index index]);
        trail.XData=[log.X_m(visible);pose(1)];trail.YData=[log.Y_m(visible);pose(2)];
        for j=1:numel(rackBorders)
            if j<=log.CompletedPicks(index),rackBorders(j).EdgeColor=[0 0.43 0.35];
            else,rackBorders(j).EdgeColor=[0.04 0.35 0.66];end
        end
        if hasPeople
            positions=(1-weight)*run.pedestrians(index,:)+weight*run.pedestrians(nextIndex,:);
            angle=linspace(0,2*pi,32);
            for h=1:numel(humanBodies)
                centre=positions(2*h-1:2*h);
                humanBodies(h).XData=centre(1)+run.scenario.radius*cos(angle);
                humanBodies(h).YData=centre(2)+run.scenario.radius*sin(angle);
                humanLabels(h).Position=[centre 0];
            end
        end
        if isfield(run,'routeUpdates') && ~isempty(run.routeUpdates)
            active=find([run.routeUpdates.time_s]<=simTime & [run.routeUpdates.leg]==log.Leg(index),1,'last');
            if isempty(active),updatedRoute.XData=nan;updatedRoute.YData=nan;
            else,updatedRoute.XData=run.routeUpdates(active).path(:,1);updatedRoute.YData=run.routeUpdates(active).path(:,2);end
            replanLabel.String=sprintf('Route updates: %d',sum([run.routeUpdates.time_s]<=simTime));
        else,replanLabel.String='Route updates: 0';end
        phaseLabel.String=upper(strrep(char(log.Phase(index)),'pedestrian_',''));
        targetLabel.String=['Station: ' char(log.Target(index))];
        picksLabel.String=sprintf('Picks completed: %d / %d',log.CompletedPicks(index),numel(run.plan.rackIds));
        timeLabel.String=sprintf('Time: %.1f s',simTime);
        distanceLabel.String=sprintf('Distance: %.2f m',log.Distance_m(index));
        speedLabel.String=sprintf('Speed: %.2f m/s',log.Speed_mps(index));
        drawnow limitrate nocallbacks;
    end
    function togglePause(~,~)
        if index==height(log),return;end
        if paused,lastTick=tic;pauseButton.String='Pause';
        else,pauseButton.String='Resume';end
        paused=~paused;
    end
    function restart(~,~)
        simTime=0;index=1;paused=false;pauseButton.String='Pause';
        render();lastTick=tic;
    end
    function changeRate(~,~)
        lastTick=tic;
    end
    function exportFigure(~,~)
        warehouse.view.exportExecution(run,folder);
        lastTick=tic;
        fprintf('Mission figure exported to:\n%s\n',folder);
    end
    function closeViewer(~,~)
        delete(fig);
    end
end

