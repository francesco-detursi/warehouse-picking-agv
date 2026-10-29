function folder = saveRun(run,root)
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
folder=fullfile(root,'results',['mission_' stamp]);suffix=1;
while isfolder(folder)
    folder=fullfile(root,'results',sprintf('mission_%s_%d',stamp,suffix));suffix=suffix+1;
end
mkdir(folder);
if isfield(run,'replans'),writetable(run.replans,fullfile(folder,'replanning.csv'));end
save(fullfile(folder,'mission_run.mat'),'run');
writetable(run.trajectory,fullfile(folder,'trajectory.csv'));
writetable(run.events,fullfile(folder,'events.csv'));
writetable(run.metrics,fullfile(folder,'metrics.csv'));
if isfield(run,'crowdSummary'),writetable(run.crowdSummary,fullfile(folder,'pedestrian_summary.csv'));end
if isfield(run,'supervisor')
 writetable(run.supervisor,fullfile(folder,'supervisor.csv'));
 writetable(run.supervisorEvents,fullfile(folder,'supervisor_events.csv'));
 names={'Time_s'};
 for k=1:numel(run.scenario.people)
  names=[names,{sprintf('H%d_X_m',k),sprintf('H%d_Y_m',k)}];
 end
 writetable(array2table([run.trajectory.Time_s run.pedestrians],'VariableNames',names),fullfile(folder,'pedestrians.csv'));
end
end
