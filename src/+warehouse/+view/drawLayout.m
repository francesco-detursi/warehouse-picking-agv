function drawLayout(fig,map,cfg,showApproaches)
ink = [0.12 0.18 0.23]; muted = [0.40 0.47 0.53];
blue = [0.04 0.35 0.66]; green = [0.00 0.43 0.35]; amber = [0.66 0.36 0.04];
ax = axes(fig,'Position',[0.055 0.15 0.68 0.70],'Color',[0.965 0.975 0.982], ...
    'FontName','Arial','FontSize',10,'XColor',muted,'YColor',muted, ...
    'LineWidth',1.1,'Box','on','Tag','WarehouseMap');
hold(ax,'on'); axis(ax,'equal');
xlim(ax,[0 24]); ylim(ax,[0 18]); xticks(ax,0:2:24); yticks(ax,0:2:18);
xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
ax.Toolbar.Visible = 'off'; disableDefaultInteractivity(ax);
for k = 1:size(map.obstacles,1)
    r = map.obstacles(k,:);
    rectangle(ax,'Position',r,'FaceColor',[0.29 0.36 0.40],'EdgeColor',ink,'LineWidth',1.3);
    for row = 0:2
        for col = 0:1
            rectangle(ax,'Position',[r(1)+0.12+col*0.94 r(2)+0.12+row*0.94 0.82 0.82], ...
                'FaceColor',[0.72 0.66 0.52],'EdgeColor',[0.85 0.84 0.77],'LineWidth',0.5);
        end
    end
    text(ax,r(1)+r(3)/2,r(2)+r(4)/2,map.rackIds(k),'HorizontalAlignment','center', ...
        'Color','white','FontName','Arial','FontSize',10,'FontWeight','bold', ...
        'BackgroundColor',[0.29 0.36 0.40],'Margin',3,'Interpreter','none');
end
for k = 1:numel(map.docks)
    d = map.docks(k);
    c = amber;
    if d.kind=="waiting", c=blue; elseif d.kind=="unloading", c=green; end
    rectangle(ax,'Position',d.zone,'FaceColor',0.92+0.08*c,'EdgeColor',c, ...
        'LineStyle','--','LineWidth',1.2);
    if d.kind=="loading"
        text(ax,d.pose(1),d.zone(2)+d.zone(4)+0.25,d.id,'HorizontalAlignment','center', ...
            'FontName','Arial','FontSize',10,'Color',c,'FontWeight','bold');
        quiver(ax,d.pose(1)-0.40,d.pose(2)-0.30,0.75,0,0,'Color',c, ...
            'LineWidth',1.1,'MaxHeadSize',0.8);
    else
        text(ax,d.pose(1),d.zone(2)+d.zone(4)+0.28,d.id, ...
            'HorizontalAlignment','center','VerticalAlignment','bottom', ...
            'Color',c,'FontSize',12,'FontWeight','bold');
    end
    if showApproaches
        plot(ax,[d.approach(1) d.pose(1)],[d.approach(2) d.pose(2)],':', ...
            'Color',muted,'LineWidth',1,'Tag','Approach');
        plot(ax,d.approach(1),d.approach(2),'o','MarkerSize',4, ...
            'MarkerFaceColor','white','MarkerEdgeColor',muted,'Tag','Approach');
    end
end
pose = map.docks(map.waitingIndex).pose;
p = warehouse.geometry.robotPolygon(pose,cfg);
patch(ax,p(:,1),p(:,2),blue,'EdgeColor',ink,'LineWidth',1.2);
plot(ax,[pose(1) pose(1)+0.38],[pose(2) pose(2)],'w-','LineWidth',2);
heading('WAREHOUSE PICKING ROBOT',0.925,20,ink);
heading('Warehouse A  /  Single-robot layout',0.875,11,muted);
side('STATIONS',0.81,12,ink,true);
side('P01-P06   Loading docks',0.755,11,amber,false);
side('D               Unloading',0.710,11,green,false);
side('W              Waiting',0.665,11,blue,false);
side('R01-R06   Storage racks',0.610,11,ink,false);
side('P01 serves R01, and so on.',0.563,9,muted,false);
side('LAYOUT',0.475,12,ink,true);
side('Warehouse         24 x 18 m',0.425,10,ink,false);
side('Robot                  1.00 x 0.70 m',0.382,10,ink,false);
side('Loading docks    1.80 x 1.50 m',0.339,10,ink,false);
side('Design margin    0.15 m',0.296,10,ink,false);
side('MISSION CYCLE',0.215,12,ink,true);
side('W > selected racks > D > W',0.169,10,ink,false);
annotation(fig,'textbox',[0.055 0.055 0.90 0.035], ...
    'String','Dashed boxes: operating zones, not obstacles  |  Arrows: docking heading  |  SI units', ...
    'EdgeColor','none','FontName','Arial','FontSize',9,'Color',muted,'Interpreter','none');
    function heading(s,y,fs,color)
        annotation(fig,'textbox',[0.055 y 0.90 0.045],'String',s,'EdgeColor','none', ...
            'FontName','Arial','FontSize',fs,'Color',color,'FontWeight',weight(fs>=20), ...
            'Interpreter','none','FitBoxToText','off');
    end
    function side(s,y,fs,color,bold)
        annotation(fig,'textbox',[0.76 y 0.235 0.043],'String',s,'EdgeColor','none', ...
            'FontName','Arial','FontSize',fs,'Color',color,'FontWeight',weight(bold), ...
            'Interpreter','none','FitBoxToText','off');
    end
end
function w = weight(bold)
if bold, w='bold'; else, w='normal'; end
end
