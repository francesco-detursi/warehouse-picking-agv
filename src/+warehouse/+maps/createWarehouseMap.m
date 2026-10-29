function map = createWarehouseMap()
map.name = "Warehouse A";
map.bounds = [0 0 24 18];
map.obstacles = [5 4 2 3;5 11 2 3;11 4 2 3;11 11 2 3;17 4 2 3;17 11 2 3];
map.rackIds = compose("R%02d",(1:6)');
map.docks = repmat(struct('id',"",'rackId',"",'kind',"", ...
    'pose',zeros(1,3),'approach',zeros(1,2),'zone',zeros(1,4)),8,1);
for k = 1:6
    rack = map.obstacles(k,:);
    centre = [rack(1)-1.5,rack(2)+rack(4)/2];
    map.docks(k) = makeDock(compose("P%02d",k),map.rackIds(k),"loading", ...
        [centre 0],centre-[1.25 0],[1.8 1.5]);
end
map.docks(7) = makeDock("D","","unloading",[22 15 pi/2],[22 13.5],[1.8 1.8]);
map.docks(8) = makeDock("W","","waiting",[22 2 0],[20.5 2],[1.8 1.8]);
map.dispatchIndex = 7;
map.waitingIndex = 8;
end

function dock = makeDock(id,rackId,kind,pose,approach,sizeXY)
dock = struct('id',id,'rackId',rackId,'kind',kind,'pose',pose, ...
    'approach',approach,'zone',[pose(1:2)-sizeXY/2 sizeXY]);
end
