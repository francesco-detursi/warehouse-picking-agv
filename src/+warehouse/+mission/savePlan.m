function folder = savePlan(plan,map,cfg,root)
requestFile=warehouse.mission.saveRequest(plan.request,root);
folder=fileparts(requestFile);
save(fullfile(folder,'mission_plan.mat'),'plan','map','cfg');
legs=plan.legs;
summary=table((1:numel(legs))',[legs.from]',[legs.to]',[legs.distance_m]', ...
    'VariableNames',{'Leg','From','To','PlannedDistance_m'});
writetable(summary,fullfile(folder,'mission_legs.csv'));
points=zeros(0,4);
for k=1:numel(legs)
    p=legs(k).path;
    points=[points;repmat(k,size(p,1),1),(1:size(p,1))',p]; %#ok<AGROW>
end
writetable(array2table(points,'VariableNames',{'Leg','Waypoint','X_m','Y_m'}),fullfile(folder,'planned_path.csv'));
end
