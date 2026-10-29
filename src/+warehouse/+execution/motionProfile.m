function profile = motionProfile(distance,maxSpeed,acceleration,deceleration,dt)
validateattributes(distance,{'numeric'},{'scalar','finite','nonnegative'});
validateattributes([maxSpeed acceleration deceleration dt],{'numeric'},{'positive','finite'});
if distance<1e-12,profile=zeros(0,3);return;end
peak=min(maxSpeed,sqrt(2*distance/(1/acceleration+1/deceleration)));
ta=peak/acceleration; td=peak/deceleration;
cruise=max(0,(distance-peak^2/(2*acceleration)-peak^2/(2*deceleration))/peak);
duration=ta+cruise+td;
t=unique([0:dt:duration ta ta+cruise duration])';
t=t([true;diff(t)>1e-10]);t(end)=duration;
s=zeros(size(t));v=s;
a=t<=ta; b=t>ta & t<=ta+cruise; c=t>ta+cruise;
s(a)=0.5*acceleration*t(a).^2;v(a)=acceleration*t(a);
s(b)=0.5*peak*ta+peak*(t(b)-ta);v(b)=peak;
remaining=duration-t(c);
s(c)=distance-0.5*deceleration*remaining.^2;v(c)=deceleration*remaining;
profile=[t s v];
end
