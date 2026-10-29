function file = saveRequest(request,root)
parent=fullfile(root,'results');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
folder=fullfile(parent,['request_' stamp]);
suffix=1;
while isfolder(folder)
    folder=fullfile(parent,sprintf('request_%s_%d',stamp,suffix)); suffix=suffix+1;
end
mkdir(folder);
file=fullfile(folder,'mission_request.json');
[fid,message]=fopen(file,'w','n','UTF-8');
assert(fid~=-1,'Warehouse:SaveFailed','Cannot save mission: %s',message);
cleanup=onCleanup(@()fclose(fid));
payload=request;
payload.requestedRackIds=cellstr(request.requestedRackIds);
fprintf(fid,'%s\n',jsonencode(payload,'PrettyPrint',true));
end
