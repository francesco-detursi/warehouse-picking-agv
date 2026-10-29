function polygon = robotPolygon(pose,cfg)
L = cfg.robot.length/2; W = cfg.robot.width/2;
local = [L W;-L W;-L -W;L -W];
R = [cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
polygon = local*R' + pose(1:2);
end
