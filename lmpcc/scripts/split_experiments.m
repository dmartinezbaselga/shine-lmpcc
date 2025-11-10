%% Splits experiments into separate structs
% Trigger is based on "reset" signal!
function [new_data] = split_experiments(data)
    reset = data.("reset");
%     reset_times = find(reset == 1, 1e3, 'first');
%     reset_times = [1 reset_times numel(reset)];
    reset_times = [1 (data.("reset")+1)];

    % Handle the case where the reset is at the last iteration
    if(numel(reset_times) > 2)
        if(reset_times(end) == reset_times(end - 1))
            reset_times = reset_times(1:end - 1);
        end
    end


    fn = fieldnames(data);

    new_data = cell(numel(reset_times) - 1, 1);

    for experiment = 1: numel(reset_times) - 1
        new_data{experiment} = struct();

        start_k = reset_times(experiment);
        stop_k = reset_times(experiment + 1);

        for i = 1 : numel(fn)
%             fn{i}
            if(size(data.(fn{i}), 2) > 1)
                try
                    new_data{experiment}.(fn{i}) = data.(fn{i})(:, start_k : stop_k-1);
                catch
                    if(strcmp(fn{i}, 'reset') == 0)
                        fprintf('Could not retrieve %s for experiment %d\n', fn{i}, experiment);
                    end
                end
            else
                new_data{experiment}.(fn{i}) = data.(fn{i});
            end
        end

    end

    % Remove data with short durations
    j = 1;
    data = {};
    for experiment = 1 : numel(new_data)
        if(size(new_data{experiment}.vehicle_pose, 2) > 20)
            data{j} = new_data{experiment};
            j = j + 1;
        end
    end
    new_data = data';

end