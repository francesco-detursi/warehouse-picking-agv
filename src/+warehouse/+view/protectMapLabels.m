function protectMapLabels(ax)
labels=findobj(ax,'Type','text');
for k=1:numel(labels)
 if startsWith(string(labels(k).String),"P0")
  labels(k).BackgroundColor=[0.965 0.975 0.982];labels(k).Margin=1;
 end
end
uistack(labels,'top');
end
