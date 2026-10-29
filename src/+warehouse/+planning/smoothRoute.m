function path = smoothRoute(route,map,cfg)
spacing=0.025;radius=cfg.robot.radius+cfg.robot.margin;
path=route(1,:);
for k=2:size(route,1)-1
    u=route(k,:)-route(k-1,:);v=route(k+1,:)-route(k,:);
    lu=norm(u);lv=norm(v);u=u/lu;v=v/lv;
    if dot(u,v)>0.999999,continue;end
    trim=min([1.4,0.42*lu,0.42*lv]);accepted=false;
    for attempt=1:7
        a=route(k,:)-trim*u;b=route(k,:)+trim*v;
        control=[a;a+0.4*trim*u;a+0.8*trim*u;b-0.8*trim*v;b-0.4*trim*v;b];
        t=linspace(0,1,max(21,ceil(4*trim/spacing)))';curve=zeros(numel(t),2);
        for j=0:5,curve=curve+nchoosek(5,j)*(1-t).^(5-j).*t.^j.*control(j+1,:);end
        step=max(vecnorm(diff(curve),2,2));
        if all(warehouse.geometry.isFree(curve,map,radius+step/2))
            accepted=true;break
        end
        trim=trim/2;
    end
    if accepted
        path=[path;linePoints(path(end,:),a,spacing);curve(2:end,:)];
    else
        path=[path;linePoints(path(end,:),route(k,:),spacing)];
    end
end
path=[path;linePoints(path(end,:),route(end,:),spacing)];
path=path([true;vecnorm(diff(path),2,2)>1e-9],:);
end
function p=linePoints(a,b,h)
n=max(1,ceil(norm(b-a)/h));p=a+(1:n)'/n.*(b-a);
end
