function run=simulatePedestrianMission(plan,map,cfg,scenario)
if nargin<4,scenario=warehouse.safety.pedestrianScenario();end
motion=warehouse.config.motionSettings();
run=warehouse.execution.simulateMission(plan,map,cfg,motion,scenario);
end
