function motion = motionSettings()
motion.maxSpeed=0.80;
motion.acceleration=0.40;
motion.deceleration=0.60;
motion.maxYawRate=0.70;
motion.yawAcceleration=1.00;
motion.sampleTime=0.02;
motion.loadingTime=8.0;
motion.unloadingTime=12.0;
motion.positionTolerance=0.025;
motion.headingTolerance=deg2rad(1);
motion.dockingSpeed=0.20;
motion.dockingDistance=0.80;
motion.lateralAcceleration=0.30;
motion.trackingGains=[1.5 4.0 3.0];
end
