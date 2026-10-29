function project = startWarehouse()
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src'));
cfg = warehouse.config.robotSettings();
map = warehouse.maps.createWarehouseMap();
checks = warehouse.validation.validateLayout(map,cfg);
assert(all(checks.Passed),'Warehouse:InvalidLayout','Layout checks failed.');
fig = warehouse.view.openMissionSetup(map,cfg,root);
project = struct('Map',map,'Config',cfg,'Checks',checks,'Figure',fig);
fprintf('\nWarehouse Picking Robot\n');
fprintf('Click racks to prepare a mission.\n');
fprintf('Plan mission, then Run mission to simulate the complete collection cycle.\n');
end
