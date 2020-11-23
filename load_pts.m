function [ pts ] = load_pts( file_path )
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
    pts = reshape(fscanf(fileID,'%f'),7,num_verteces);
    fclose(fileID);
end

