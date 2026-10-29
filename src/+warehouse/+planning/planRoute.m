function route = planRoute(start, goal, map, cfg)
radius = cfg.robot.radius + cfg.robot.margin;
spacing = cfg.planner.resolution;
route = zeros(0,2);
if ~all(warehouse.geometry.isFree([start;goal],map,radius))
    return
end
xs = map.bounds(1):spacing:map.bounds(1)+map.bounds(3);
ys = map.bounds(2):spacing:map.bounds(2)+map.bounds(4);
[X,Y] = meshgrid(xs,ys);
points = [X(:) Y(:)];
allowed = reshape(warehouse.geometry.isFree(points,map,radius),size(X));
s = nearestConnected(start,points,allowed,map,radius,spacing);
g = nearestConnected(goal,points,allowed,map,radius,spacing);
if isempty(s) || isempty(g), return; end
n = numel(X);
cost = inf(n,1); cost(s) = 0;
score = inf(n,1); score(s) = norm(points(s,:)-points(g,:));
parent = zeros(n,1);
open = false(n,1); open(s) = true;
closed = false(n,1);
offsets = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
while any(open)
    candidates = find(open);
    [~,j] = min(score(candidates));
    u = candidates(j);
    if u == g, break; end
    open(u) = false; closed(u) = true;
    [row,col] = ind2sub(size(X),u);
    for k = 1:8
        rr = row + offsets(k,1); cc = col + offsets(k,2);
        if rr<1 || rr>size(X,1) || cc<1 || cc>size(X,2), continue; end
        v = sub2ind(size(X),rr,cc);
        if closed(v) || ~allowed(v), continue; end
        if ~warehouse.geometry.segmentFree(points(u,:),points(v,:),map,radius,spacing/4), continue; end
        trial = cost(u) + norm(points(v,:)-points(u,:));
        if trial < cost(v)
            cost(v) = trial;
            score(v) = trial + norm(points(v,:)-points(g,:));
            parent(v) = u; open(v) = true;
        end
    end
end
if ~isfinite(cost(g)), return; end
indices = g;
while indices(1) ~= s
    indices = [parent(indices(1));indices]; %#ok<AGROW>
end
raw = [start;points(indices,:);goal];
raw = raw([true;vecnorm(diff(raw),2,2)>1e-10],:);
route = raw(1,:);
i = 1;
while i < size(raw,1)
    j = size(raw,1);
    while j>i+1 && ~warehouse.geometry.segmentFree(raw(i,:),raw(j,:),map,radius,spacing/4)
        j = j-1;
    end
    route(end+1,:) = raw(j,:); %#ok<AGROW>
    i = j;
end
end

function index = nearestConnected(p,points,allowed,map,radius,spacing)
[~,order] = sort(sum((points-p).^2,2));
index = [];
for k = 1:numel(order)
    j = order(k);
    if allowed(j) && warehouse.geometry.segmentFree(p,points(j,:),map,radius,spacing/4)
        index = j;
        return
    end
end
end
