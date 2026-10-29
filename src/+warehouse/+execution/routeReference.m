function ref = routeReference(path,motion)
if size(path,1)==2
    delta=path(2,:)-path(1,:);length=norm(delta);
    profile=warehouse.execution.motionProfile(length,min(0.95*motion.maxSpeed,motion.dockingSpeed), ...
        0.8*motion.acceleration,0.8*motion.deceleration,motion.sampleTime);
    ref=[profile(:,1),path(1,:)+profile(:,2)/length.*delta, ...
        repmat(atan2(delta(2),delta(1)),size(profile,1),1),profile(:,3),zeros(size(profile,1),1)];
    return
end
s=[0;cumsum(vecnorm(diff(path),2,2))];
heading=unwrap(atan2(gradient(path(:,2),s),gradient(path(:,1),s)));
kappa=gradient(heading,s);
v=min(0.95*motion.maxSpeed*ones(size(s)),sqrt(motion.lateralAcceleration./max(abs(kappa),1e-6)));
v=min(v,0.8*motion.maxYawRate./max(abs(kappa),1e-6));
v(s(end)-s<=motion.dockingDistance)=min(v(s(end)-s<=motion.dockingDistance),motion.dockingSpeed);
dk=gradient(kappa,s);
v=min(v,sqrt(0.30*motion.yawAcceleration./max(abs(dk),1e-6)));
aLimit=min(0.8*motion.acceleration,0.30*motion.yawAcceleration./max(abs(kappa),1e-6));
dLimit=min(0.8*motion.deceleration,0.30*motion.yawAcceleration./max(abs(kappa),1e-6));
v(1)=0;v(end)=0;
for k=2:numel(v),v(k)=min(v(k),sqrt(v(k-1)^2+2*min(aLimit(k-1:k))*(s(k)-s(k-1))));end
for k=numel(v)-1:-1:1,v(k)=min(v(k),sqrt(v(k+1)^2+2*min(dLimit(k:k+1))*(s(k+1)-s(k))));end
t=[0;cumsum(2*diff(s)./(v(1:end-1)+v(2:end)))];
% Time dilation reserves angular acceleration and feedback authority.
omega=kappa.*v;
scale=max([1;sqrt(max(abs(gradient(omega,t)))/(0.65*motion.yawAcceleration))]);
t=t*scale;v=v/scale;
tq=(0:motion.sampleTime:t(end))';
if t(end)-tq(end)>1e-9,tq(end+1)=t(end);else,tq(end)=t(end);end
segment=discretize(tq,[t(1:end-1);inf]);dt=tq-t(segment);
a=diff(v)./diff(t);a=[a;0];
sq=s(segment)+v(segment).*dt+0.5*a(segment).*dt.^2;sq(end)=s(end);
xy=interp1(s,path,sq,'pchip');th=interp1(s,heading,sq,'pchip');
vq=interp1(t,v,tq);wq=interp1(s,kappa,sq).*vq;
vq([1 end])=0;wq([1 end])=0;
ref=[tq xy th vq wq];
end
