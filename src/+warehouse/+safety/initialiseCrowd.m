function crowd=initialiseCrowd(scenario)
n=numel(scenario.people);crowd.position=vertcat(scenario.people.points);
crowd.velocity=zeros(n,2);crowd.target=zeros(n,1);crowd.route=cell(n,1);
crowd.pause=zeros(n,1);crowd.visits=zeros(n,1);crowd.distance=zeros(n,1);
crowd.yieldTime=zeros(n,1);crowd.emergencyStops=zeros(n,1);
crowd.detour=cell(n,1);crowd.bestDistance=inf(n,1);
crowd.stall=zeros(n,1);crowd.redirections=zeros(n,1);
crowd.stream=cell(n,1);
for k=1:n
 crowd.target(k)=find(all(scenario.nodes==crowd.position(k,:),2),1);
 crowd.stream{k}=RandStream('mt19937ar','Seed',mod(scenario.seed+7919*k,2^32));
 crowd.pause(k)=2*rand(crowd.stream{k});
end
end
