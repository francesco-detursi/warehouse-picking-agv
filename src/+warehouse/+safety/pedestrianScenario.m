function scenario=pedestrianScenario()
scenario.name="Warehouse crossings";
scenario.radius=0.30;
scenario.separation=0.30;
scenario.sensorRange=8.0;
scenario.predictionTime=3.0;
scenario.clearTime=1.0;
scenario.crossingZones=[19.6 1.6 0.8 4.8;19.6 5.6 3.0 0.8];
scenario.scaleRate=0.25;
scenario.maxDuration=1800;
scenario.people(1)=struct('id',"H1",'points',[20 2;20 6;23.5 6], ...
    'times',[0 16 22],'period',1000000,'hold',8);
end
