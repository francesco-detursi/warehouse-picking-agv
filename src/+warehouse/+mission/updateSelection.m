function state = updateSelection(state,action,rackCount,value)
if nargin<4, value=[]; end
switch string(action)
    case "toggle"
        validateattributes(value,{'numeric'},{'scalar','integer','>=',1,'<=',rackCount});
        row = find(state.rackIndices==value,1);
        if isempty(row)
            state.rackIndices(end+1)=value;
            state.focus=numel(state.rackIndices);
        else
            state.rackIndices(row)=[];
            state.focus=min(row,numel(state.rackIndices));
        end
    case "focus"
        validateattributes(value,{'numeric'},{'scalar','integer','>=',1,'<=',numel(state.rackIndices)});
        state.focus=value;
    case "reorder"
        if state.orderMode=="manual" && state.focus>0
            validateattributes(value,{'numeric'},{'scalar','integer','>=',1,'<=',numel(state.rackIndices)});
            item=state.rackIndices(state.focus);
            state.rackIndices(state.focus)=[];
            state.rackIndices=[state.rackIndices(1:value-1) item state.rackIndices(value:end)];
            state.focus=value;
        end
    case "remove"
        if state.focus>0
            state.rackIndices(state.focus)=[];
            state.focus=min(state.focus,numel(state.rackIndices));
        end
    case "clear"
        state.rackIndices=zeros(1,0);
        state.focus=0;
    case "mode"
        assert(any(string(value)==["manual","optimised"]),'Warehouse:InvalidMode','Unknown ordering mode.');
        state.orderMode=string(value);
    otherwise
        error('Warehouse:InvalidAction','Unknown selection action.');
end
end
