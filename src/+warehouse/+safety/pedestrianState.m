function [positions,velocity]=pedestrianState(time,scenario)
n=numel(scenario.people);positions=zeros(n,2);velocity=zeros(n,2);
for k=1:n
 p=scenario.people(k);t=mod(time,p.period);
 if t<=p.hold,positions(k,:)=p.points(1,:);continue;end
 j=find(p.times<=t,1,'last');
 if j>=numel(p.times),positions(k,:)=p.points(end,:);continue;end
 a=p.times(j);if j==1,a=p.hold;end
 duration=p.times(j+1)-a;
 velocity(k,:)=(p.points(j+1,:)-p.points(j,:))/duration;
 positions(k,:)=p.points(j,:)+(t-a)*velocity(k,:);
end
end
