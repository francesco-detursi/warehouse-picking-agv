function request = createRequest(state,map)
assert(~isempty(state.rackIndices),'Warehouse:EmptySelection','Select at least one rack.');
indices=state.rackIndices;
validateattributes(indices,{'numeric'},{'vector','integer','>=',1,'<=',numel(map.rackIds)});
assert(numel(unique(indices))==numel(indices),'Warehouse:DuplicateRack','A rack can only be selected once.');
assert(any(state.orderMode==["manual","optimised"]),'Warehouse:InvalidMode','Unknown ordering mode.');
request.schemaVersion=1;
request.mapName=map.name;
request.status="draft";
request.orderMode=state.orderMode;
request.requestedRackIds=reshape(map.rackIds(indices),1,[]);
request.startStation=map.docks(map.waitingIndex).id;
request.unloadingStation=map.docks(map.dispatchIndex).id;
request.returnStation=map.docks(map.waitingIndex).id;
end
