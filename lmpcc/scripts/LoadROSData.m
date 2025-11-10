%% Move this file to something in your path
%% Exports a structure
function [data] = LoadROSData(filename)
    fid = fopen(filename);

    format_spec = '%s';
    for i = 1 : 100000
        entry = fscanf(fid, '%s: %d %d', 1);
        var_name = char(entry(1:end-1)');
        if(var_name == '-')
            break;
        end

        num_per_var = fscanf(fid, '%d', 1);
        total_num = fscanf(fid, '%d', 1);

        data.(var_name) = zeros(num_per_var, total_num);

        data.(var_name) = fscanf(fid, '%f', [num_per_var, total_num]);
    end

    fclose(fid);
end
