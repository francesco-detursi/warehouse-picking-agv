function exportMotion(run,folder)
f=figure('Visible','off','Color','white','Position',[50 50 1100 750]);cleanup=onCleanup(@()close(f));
tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');q=run.trajectory;
a=nexttile;plot(a,q.Time_s,q.Speed_mps,'Color',[0.04 0.35 0.66]);ylabel(a,'Speed [m/s]');grid(a,'on');title(a,'Mission motion profile');
a=nexttile;plot(a,q.Time_s,q.YawRate_radps,'Color',[0 0.43 0.35]);ylabel(a,'Yaw rate [rad/s]');grid(a,'on');
a=nexttile;plot(a,q.Time_s,1000*q.TrackingError_m,'Color',[0.66 0.36 0.04]);ylabel(a,'Tracking error [mm]');xlabel(a,'Simulation time [s]');grid(a,'on');
exportgraphics(f,fullfile(folder,'motion_profile.png'),'Resolution',200);
exportgraphics(f,fullfile(folder,'motion_profile.pdf'),'ContentType','vector');
end
