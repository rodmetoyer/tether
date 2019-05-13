function filelist = getFilesList(fldrpath,extension)
% Retrieves a list of input files with the desired extension
% INPUTS:
    % fldrpath = the full path to the folder containing the files
    % extension = the extension for the desired files
% OUTPUTS
    % filelist = a list of files
listing    = dir(fldrpath);
%nlist1     = size({listing(1:end).name}',1);
filelist       = {listing(1:end).name}';
nlist      = size(filelist,1);
for j = 1 : nlist
    if isempty(strfind(filelist{j},extension)) == 1
        idx = j;
        filelist{j} = [];
    end
 end
filelist(cellfun('isempty',filelist)) = []; % Remove empty cells
nlist      = size(filelist,1);
for i = 1 : nlist
    if isempty(strfind(filelist{i},extension)) == 1 %selecting files with '.txt' in it
        idx = i;
        filelist{i} = []; %list with .jpg are set to [] or empty cells
    end
end
filelist(cellfun('isempty',filelist)) = []; %removing the empty cells
nlist = size(filelist,1);
if nlist < 1
    error(['No ' extension ' files found in ' fldrpath]);
end
end