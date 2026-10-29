function scenario=randomPedestrianScenario(map,seed,count)
if nargin<2,seed=42;end
if nargin<3,count=3;end
validateattributes(seed,{'numeric'},{'scalar','integer','>=',0,'<=',2^32-1});
validateattributes(count,{'numeric'},{'scalar','integer','>=',1,'<=',5});
scenario=warehouse.safety.pedestrianScenario();
scenario.kind="random";scenario.name="Random pedestrian traffic";
scenario.seed=double(seed);scenario.crossingZones=zeros(0,4);
scenario.showRoutes=false;scenario.maxDuration=1200;
scenario.walkAcceleration=0.8;scenario.personalGap=0.15;
scenario.robotGap=0.45;scenario.walkPrediction=1.5;
x=[3 9 15 21];y=[2 5.5 8.8 12.5 16];
[X,Y]=ndgrid(x,y);scenario.nodes=[X(:) Y(:)];
n=size(scenario.nodes,1);adj=zeros(n);
for a=1:n
 for b=a+1:n
  p=scenario.nodes(a,:);q=scenario.nodes(b,:);
  adjacent=(p(1)==q(1) && abs(find(y==p(2))-find(y==q(2)))==1) || ...
      (p(2)==q(2) && any(p(2)==y([1 3 5])) && abs(p(1)-q(1))==6);
  if adjacent,adj(a,b)=norm(p-q);adj(b,a)=adj(a,b);end
 end
end
scenario.graph=graph(adj);
stream=RandStream('mt19937ar','Seed',seed);
starts=randperm(stream,n,count);
waiting=map.docks(map.waitingIndex).pose;
allowed=find(vecnorm(scenario.nodes-waiting(1:2),2,2)>=1.5);
for k=1:count
 if ~ismember(starts(k),allowed)
  choices=setdiff(allowed,starts);starts(k)=choices(randi(stream,numel(choices)));
 end
end
scenario.people=repmat(struct('id',"",'points',zeros(1,2),'times',0,'period',1e9,'hold',0,'speed',0),count,1);
for k=1:count
 scenario.people(k)=struct('id',"H"+k,'points',scenario.nodes(starts(k),:), ...
  'times',0,'period',1e9,'hold',0,'speed',0.65+0.35*rand(stream));
end
for k=1:n
 p=scenario.nodes(k,:);d=max(max(map.obstacles(:,1:2)-p,0),p-map.obstacles(:,1:2)-map.obstacles(:,3:4));
 assert(all(vecnorm(d,2,2)>scenario.radius+0.1),'Warehouse:PedestrianMap','Pedestrian graph intersects a rack.');
end
end
