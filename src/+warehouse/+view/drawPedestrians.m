function [bodies,labels]=drawPedestrians(ax,scenario,positions)
n=numel(scenario.people);bodies=gobjects(n,1);labels=gobjects(n,1);
angle=linspace(0,2*pi,32);
for k=1:n
 route=scenario.people(k).points;
 if ~isfield(scenario,'showRoutes') || scenario.showRoutes
  plot(ax,route(:,1),route(:,2),':','Color',[0.85 0.55 0.35],'LineWidth',1);
 end
 bodies(k)=patch(ax,positions(k,1)+scenario.radius*cos(angle),positions(k,2)+scenario.radius*sin(angle), ...
    [0.85 0.30 0.07],'EdgeColor',[0.45 0.15 0.02],'LineWidth',1.2);
 labels(k)=text(ax,positions(k,1),positions(k,2),scenario.people(k).id, ...
    'FontName','Arial','FontSize',7.5,'FontWeight','bold','Color','white', ...
    'HorizontalAlignment','center','VerticalAlignment','middle');
end
end
