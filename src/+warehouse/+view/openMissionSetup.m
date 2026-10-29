function fig = openMissionSetup(map,cfg,root)
ink=[0.12 0.18 0.23]; muted=[0.40 0.47 0.53]; blue=[0.04 0.35 0.66];
screen=get(groot,'ScreenSize');
width=min(1240,screen(3)-80); height=min(760,screen(4)-120);
fig=figure('Name','Warehouse Picking Robot | Mission setup', ...
    'NumberTitle','off','Color','white','MenuBar','none','ToolBar','none', ...
    'Position',[40 60 width height],'Tag','WarehouseMissionSetup');
warehouse.view.drawLayout(fig,map,cfg,false);
labels=findall(fig,'Type','textboxshape');
for k=1:numel(labels)
    position=labels(k).Position;
    if position(1)>=0.75
        delete(labels(k));
    elseif contains(string(labels(k).String),'Single-robot layout')
        labels(k).String='Warehouse A  /  Mission setup';
    elseif contains(string(labels(k).String),'Dashed boxes:')
        labels(k).String='Click a rack to select or deselect it  |  Blue: collection / unloading  |  Green dashed: return to W';
    end
end
ax=findobj(fig,'Tag','WarehouseMap');
set(findall(ax,'-property','HitTest'),'HitTest','off');
set(ax,'HitTest','on','ButtonDownFcn',@clickMap);
label('PEDESTRIAN TRAFFIC',[0.765 0.92 0.22 0.03],10,true);
pedestrianMode=uicontrol(fig,'Style','popupmenu','Units','normalized','Position',[0.765 0.878 0.215 0.04], ...
    'String',{'No pedestrians','Random pedestrians'},'Value',2, ...
    'BackgroundColor','white','FontName','Arial','FontSize',10,'Tag','TrafficMode','Callback',@trafficModeChanged);
label('People',[0.765 0.834 0.05 0.03],9,false);
pedestrianCount=uicontrol(fig,'Style','popupmenu','Units','normalized','Position',[0.815 0.829 0.045 0.04], ...
    'String',{'1','2','3','4','5'},'Value',3,'BackgroundColor','white','FontName','Arial','FontSize',9,'Tag','PedestrianCount');
label('Seed',[0.872 0.834 0.04 0.03],9,false);
trafficSeed=uicontrol(fig,'Style','edit','Units','normalized','Position',[0.914 0.834 0.066 0.03], ...
    'String','42','BackgroundColor','white','FontName','Arial','FontSize',9,'Tag','TrafficSeed');
state=warehouse.mission.newSelection();
plan=[];executing=false;
label('SELECTED RACKS',[0.765 0.79 0.22 0.03],12,true);
countLabel=label('0 of 6 selected',[0.765 0.755 0.22 0.03],9,false);
list=axes(fig,'Units','normalized','Position',[0.765 0.505 0.215 0.24], ...
    'XLim',[0 1],'YLim',[0.5 6.5],'XTick',[],'YTick',[],'Box','on', ...
    'Color','white','XColor',[0.8 0.83 0.85],'YColor',[0.8 0.83 0.85], ...
    'Tag','SelectedRacks','ButtonDownFcn',@beginDrag);
list.Toolbar.Visible='off'; disableDefaultInteractivity(list); hold(list,'on');
dragging=false;
fig.WindowButtonMotionFcn=@dragRow;
fig.WindowButtonUpFcn=@finishDrag;
clearButton=button('Clear all',[0.765 0.458 0.103 0.035],@(~,~)change('clear'),'ClearAll');
remove=button('Clear selected',[0.877 0.458 0.103 0.035],@(~,~)change('remove'),'ClearSelected');
label('VISIT ORDER',[0.765 0.394 0.22 0.04],11,true);
mode=uicontrol(fig,'Style','popupmenu','Units','normalized','Position',[0.765 0.352 0.215 0.04], ...
    'String',{'Follow selected order','Optimise visit order'},'Value',1, ...
    'FontName','Arial','FontSize',10,'BackgroundColor','white','Tag','OrderMode','Callback',@changeMode);
helpLabel=label('',[0.765 0.272 0.215 0.07],9,false);
label('MISSION',[0.765 0.223 0.22 0.035],11,true);
preview=label('',[0.765 0.153 0.215 0.063],10,false);
distanceLabel=label('Planned distance: --',[0.765 0.117 0.215 0.03],10,false);
planButton=button('Plan mission',[0.765 0.080 0.103 0.038],@calculatePlan,'PlanMission');
saveButton=button('Save plan',[0.877 0.080 0.103 0.038],@saveMission,'SaveMission');
runButton=button('Run mission',[0.765 0.030 0.215 0.038],@runMission,'RunMission');
status=label('Select the racks to serve.',[0.06 0.014 0.67 0.035],9,false);
refresh();
    function trafficModeChanged(~,~)
        pedestrianCount.Enable=enabled(pedestrianMode.Value==2);
        trafficSeed.Enable=enabled(pedestrianMode.Value==2);
    end
    function handle=label(text,position,fontSize,bold)
        weight='normal'; if bold, weight='bold'; end
        handle=uicontrol(fig,'Style','text','Units','normalized','Position',position,'String',text, ...
            'HorizontalAlignment','left','FontName','Arial','FontSize',fontSize,'FontWeight',weight, ...
            'BackgroundColor','white','ForegroundColor',ink);
    end
    function handle=button(text,position,callback,tag)
        handle=uicontrol(fig,'Style','pushbutton','Units','normalized','Position',position, ...
            'String',text,'FontName','Arial','FontSize',9,'Callback',callback,'Tag',tag);
    end
    function clickMap(~,~)
        if ~strcmp(fig.SelectionType,'normal'), return; end
        point=ax.CurrentPoint; point=point(1,1:2);
        for index=1:numel(map.rackIds)
            r=map.obstacles(index,:);
            if all(point>=r(1:2)) && all(point<=r(1:2)+r(3:4))
                change('toggle',index); return
            end
        end
    end
    function beginDrag(~,~)
        if ~strcmp(fig.SelectionType,'normal'), return; end
        point=list.CurrentPoint; row=floor(6.5-point(1,2))+1;
        if row<1 || row>numel(state.rackIndices), return; end
        shown=displayOrder();
        selected=find(state.rackIndices==shown(row),1);
        state=warehouse.mission.updateSelection(state,'focus',numel(map.rackIds),selected);
        refresh();
        dragging=state.orderMode=="manual";
        if dragging, fig.Pointer='fleur'; end
    end
    function dragRow(~,~)
        if ~dragging, return; end
        point=list.CurrentPoint;
        target=max(1,min(numel(state.rackIndices),floor(6.5-point(1,2))+1));
        delete(findall(list,'Tag','DropIndicator'));
        if point(1,1)>=0 && point(1,1)<=1
            y=7-target;
            if target>state.focus, y=y-0.46; else, y=y+0.46; end
            plot(list,[0.03 0.97],[y y],'-','Color',blue,'LineWidth',2, ...
                'HitTest','off','PickableParts','none','Tag','DropIndicator');
        end
    end
    function finishDrag(~,~)
        if ~dragging, return; end
        dragging=false; fig.Pointer='arrow';
        delete(findall(list,'Tag','DropIndicator'));
        point=list.CurrentPoint;
        if point(1,1)<0 || point(1,1)>1 || point(1,2)<0.5 || point(1,2)>6.5, return; end
        target=max(1,min(numel(state.rackIndices),floor(6.5-point(1,2))+1));
        change('reorder',target);
    end
    function change(action,value)
        if nargin<2, value=[]; end
        state=warehouse.mission.updateSelection(state,action,numel(map.rackIds),value);
        plan=[];
        status.String='Selection updated.'; status.ForegroundColor=muted;
        refresh();
    end
    function changeMode(source,~)
        modes=["manual","optimised"];
        change('mode',modes(source.Value));
    end
    function refresh()
        n=numel(state.rackIndices);
        countLabel.String=sprintf('%d of %d selected',n,numel(map.rackIds));
        manual=state.orderMode=="manual";
        delete(list.Children);
        if n==0
            text(list,0.05,6,'Click racks on the map','Color',muted,'FontName','Arial', ...
                'FontSize',10,'HitTest','off','PickableParts','none');
        else
            shown=displayOrder();
            ids=map.rackIds(shown);
            for row=1:n
                y=7-row;
                if shown(row)==state.rackIndices(state.focus)
                    patch(list,[0 1 1 0],[y-0.49 y-0.49 y+0.49 y+0.49], ...
                        [0.90 0.95 0.99],'EdgeColor','none','HitTest','off','PickableParts','none');
                end
                caption=ids(row);
                if manual || ~isempty(plan), caption=compose('%d.   %s',row,caption); end
                text(list,0.07,y,caption,'Color',ink,'FontName','Arial','FontSize',11, ...
                    'HitTest','off','PickableParts','none');
                if manual
                    text(list,0.92,y,'=','Color',muted,'HorizontalAlignment','center', ...
                        'FontSize',12,'HitTest','off','PickableParts','none');
                end
            end
        end
        remove.Enable=enabled(state.focus>0); clearButton.Enable=enabled(n>0); planButton.Enable=enabled(n>0); saveButton.Enable=enabled(~isempty(plan)); runButton.Enable=enabled(~isempty(plan) && ~executing);
        if manual
            helpLabel.String='Drag rows up or down to change the visit order. Click a row to select it.';
        else
            helpLabel.String='Plan the mission to minimise distance. Manual ordering is preserved when switching back.';
        end
        if n==0
            preview.String='W > select racks > D > W';
        elseif manual || ~isempty(plan)
            ids=map.rackIds(displayOrder());
            split=min(n,3);
            first="W > "+strjoin(ids(1:split)'," > ");
            if n>split
                second=strjoin(ids(split+1:end)'," > ")+" > D > W";
            else
                second="D > W";
            end
            preview.String={char(first),['> ' char(second)]};
        else
            preview.String={'W > selected racks > D > W','Visit order: pending planning'};
        end
        distanceLabel.String='Planned distance: --';
        if ~isempty(plan), distanceLabel.String=sprintf('Planned distance: %.2f m',plan.distance_m); end
        delete(findall(ax,'Tag','PlannedRoute'));
        if ~isempty(plan)
            warehouse.view.drawMissionRoute(ax,plan);
        end
        delete(findall(ax,'Tag','SelectionOverlay'));
        shown=displayOrder();
        for row=1:n
            index=shown(row); r=map.obstacles(index,:);
            rectangle(ax,'Position',r,'EdgeColor',blue,'LineWidth',2.5, ...
                'HitTest','off','PickableParts','none','Tag','SelectionOverlay');
            badge='Selected'; if manual || ~isempty(plan), badge=sprintf('%d',row); end
            text(ax,r(1)+r(3)/2,r(2)+r(4)+0.35,badge,'Color',blue, ...
                'HorizontalAlignment','center','VerticalAlignment','bottom', ...
                'FontName','Arial','FontSize',10,'FontWeight','bold','BackgroundColor','white','Margin',1, ...
                'HitTest','off','PickableParts','none','Tag','SelectionOverlay');
        end
        setappdata(fig,'MissionSelection',state);
        setappdata(fig,'MissionPlan',plan);
    end
    function order=displayOrder()
        order=state.rackIndices;
        if state.orderMode=="optimised" && ~isempty(plan), order=plan.rackIndices; end
    end
    function calculatePlan(~,~)
        status.String='Planning the complete mission...';
        fig.Pointer='watch'; drawnow;
        try
            plan=warehouse.planning.planMission(state,map,cfg);
            status.String='Plan ready. Includes unloading and return.';
            status.ForegroundColor=blue;
        catch problem
            plan=[]; status.String='Mission cannot be planned.';
            errordlg(problem.message,'Planning failed');
        end
        fig.Pointer='arrow'; refresh();
    end
    function runMission(~,~)
        if isempty(plan) || executing,return;end
        executing=true;runButton.Enable='off';fig.Pointer='watch';status.String='Computing and checking the simulated mission...';drawnow;
        try
            if pedestrianMode.Value==2
                seed=str2double(trafficSeed.String);
                scenario=warehouse.safety.randomPedestrianScenario(map,seed,pedestrianCount.Value);
                result=warehouse.safety.simulatePedestrianMission(plan,map,cfg,scenario);
            else
                result=warehouse.execution.simulateMission(plan,map,cfg);
            end
            folder=warehouse.execution.saveRun(result,root);
            fig.Pointer='arrow';status.String='Results saved. Close replay before starting another run.';
            warehouse.view.showExecution(result,folder);
            if isgraphics(fig),status.String='Replay closed. Results saved.';end
            fprintf('\nMission results saved to:\n%s\n',folder);
        catch problem
            status.String='Mission execution failed.';errordlg(problem.message,'Execution failed');
        end
        executing=false;
        if isgraphics(fig),fig.Pointer='arrow';runButton.Enable=enabled(~isempty(plan));end
    end
    function saveMission(~,~)
        if isempty(plan), return; end
        try
            folder=warehouse.mission.savePlan(plan,map,cfg,root);
            warehouse.view.exportMissionPlan(plan,map,cfg,folder);
            setappdata(fig,'LastSavedPlan',folder);
            status.String='Plan saved.'; status.ForegroundColor=blue;
            fprintf('\nMission plan saved to:\n%s\n',folder);
        catch problem
            status.String='Could not save the plan.';
            errordlg(problem.message,'Save failed');
        end
    end

end
function value=enabled(condition)
if condition, value='on'; else, value='off'; end
end
