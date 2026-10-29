function cfg = robotSettings()
cfg.version = "1.0.0-phase1";
cfg.robot.length = 1.00;
cfg.robot.width = 0.70;
cfg.robot.margin = 0.15;
cfg.robot.radius = hypot(cfg.robot.length,cfg.robot.width)/2;
cfg.planner.resolution = 0.25;
cfg.validation.sampleSpacing = 0.05;
end
