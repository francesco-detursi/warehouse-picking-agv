function [order,distance] = chooseVisitOrder(cost,visits,start,unload,finish)
assert(~isempty(visits),'Warehouse:EmptySelection','Select at least one rack.');
assert(numel(visits)<=8,'Warehouse:TooManyVisits','Exact ordering supports up to eight visits.');
candidates=sortrows(perms(visits));
order=zeros(1,0); distance=inf;
for k=1:size(candidates,1)
    sequence=[start candidates(k,:) unload finish];
    total=sum(cost(sub2ind(size(cost),sequence(1:end-1),sequence(2:end))));
    if total<distance-1e-9
        distance=total; order=candidates(k,:);
    end
end
end
