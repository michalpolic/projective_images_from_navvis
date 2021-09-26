function [ pts ] = load_pts( file_path )
    [filepath,name,ext] = fileparts(file_path);
    if ext == '.ply'
    %LOAD_PTS - load pts from SIEMENS factory
        fileID = fopen(file_path,'r');
        line_ex = 0;
        num_verteces = 0;
        while ~strcmp(line_ex,'end_header') & line_ex ~= -1
            line_ex = fgetl(fileID);
            if length(line_ex) > 14 & strcmp('element vertex',line_ex(1:14))
                num_verteces = str2num(line_ex(16:end));
            end
        end
        f = fscanf(fileID,'%f')
        pts = reshape(fscanf(fileID,'%f'),7,num_verteces);
        fclose(fileID);
    elseif ext == '.xyz'
    % load pointcloud from .xyz from matterport
        fileID = fopen(file_path,'r');
        if isfile(fullfile(pwd,'tmp','cloud.mat'))
            load(fullfile(pwd,'tmp','cloud.mat'),'pts_all')
        else
            pts_all = fscanf(fileID,'%f');
        end

        pts = reshape(pts_all,6,length(pts_all)/6);
        len = length(pts);
        pts = [pts; ones(1,len)*255];
        fclose(fileID);
    end
end

