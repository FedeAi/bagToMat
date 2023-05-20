%% Main function to extract and save data from one or multiple rosbags

function bagToMatConverter(whitelist, blacklist, use_last_name, destination_foder)
    [files,paths] = uigetfile('*.bag', 'Select bag files', 'MultiSelect', 'on');
    file_paths = fullfile(paths, files);

    % handle single bag case:
    if ischar(file_paths)
        file_paths = {file_paths}; % Convert single string to cell array
        files = {files}; % Convert single string to cell array
    end
    % handle single bag case:
    if ischar(blacklist)
        blacklist = {blacklist}; % Convert single string to cell array
    end
    if ischar(whitelist) 
        whitelist = {whitelist}; % Convert single string to cell array
    end

    for i=1:length(file_paths)
        data = struct(); % Data structure to hold the extracted data
        if isvector(file_paths)
            file_path = file_paths{i};
            file = files{i};
        else
            file_path = file_paths;
            file = files;
        end
        bag = rosbag(file_path);
        
        bagInfo = rosbag("info", file_path);
        bag_name = strsplit(file,".");
        bag_name = bag_name{1};
        start_time = double(bagInfo.Start.Sec)*1e9 + double(bagInfo.Start.Nsec);
        start_time = 0.0;
        % Retrive list of topics
        bag_topics = cell(size(bagInfo.Topics));
        % Loop over the struct array and extract the x field from each struct
        for j = 1:numel(bag_topics)
            bag_topics{j} = bagInfo.Topics(j).Topic;
        end
        % Itereate trough al topics and load the messages
        
        for j=1:length(bag_topics)
            if(isempty(blacklist) || ~contains(bag_topics(j), blacklist)) && (isempty(whitelist) || contains(bag_topics(j), whitelist))
                data = getMatFromTopic(bag, bag_topics(j), start_time, use_last_name, "", data);
            end
        end
        output_file = fullfile(destination_foder, bag_name + ".mat");
        % Create the directory if it doesn't exist
        if ~isfolder(destination_foder)
            mkdir(destination_foder);
        end
        save(output_file, '-struct', 'data'); % Save the data structure to a .mat file
    end

end


%% Data Extraction Function

function data = getData( msg_struct, tstamps_rel, base_name, only_last_name, data)

    topic_overhead = isfield(msg_struct, "MessageType") + isfield(msg_struct, "Header") + 1;
    name = string(fieldnames(msg_struct));
    struct_new = struct2cell(msg_struct);
    
    for i=topic_overhead:length(name)
        name_new(i) = name(i);
        if (base_name ~= "")
            name_new(i) = base_name + "_" + name_new(i);
        end
       
        % Struct case
        if length(cell2mat(struct_new(i,1))) == 1 && isstruct(struct_new{i,1}) 
            % recursive call
            for j=1:length(struct_new(i,:))
                data = getData(struct_new{i,j}, tstamps_rel, name_new(i), only_last_name, data);
            end
            continue;
        
        % Scalar case
        elseif (length(cell2mat(struct_new(i,1))) == 1) 
            temp = cell2mat(struct_new(i,:));
        
        % Vector case
        else
            temp = [];
            for j=1:length(struct_new(i,:))
                out = cell2mat(struct_new(i,j));
                if (isstruct(out))
                    % recursive call                
                    data = getData(out, tstamps_rel, name_new(i), only_last_name, data);
                    continue;
                end
                % take as temp variable the full vector
                temp = [temp; out'];
            end
            % Check if timestamps length matches the array length
            if length(tstamps_rel) == length(temp)
                var_temp = [tstamps_rel temp];
            else
                var_temp = temp;
            end
            
            x = char(name_new(i));
            if regexp(x, '^\d') % Check if the string starts with a digit
                x = ['f' x]; % Add 'f' in front of the string
            end
            % Append data to the data structure
            if isfield(data, x)
                data.(x) = [data.(x); var_temp];
            else
                data.(x) = var_temp;
            end

            if length(tstamps_rel) == length(data.(x))
                data.(x) = [tstamps_rel data.(x)];
            end
            continue;
        end

        x = char(name_new(i));
        if regexp(x, '^\d') % Check if the string starts with a digit
            x = ['f' x]; % Add 'f' in front of the string
        end

        var_temp = double(temp');
        % Append data to the data structure
        if isfield(data, x)
            data.(x) = [data.(x); var_temp];
        else
            data.(x) = var_temp;
        end

        if length(tstamps_rel) == length(data.(x))
            data.(x) = [tstamps_rel data.(x)];
        end
    end
end



%% Main Function for extracting data from a single topic

function data = getMatFromTopic(bag, Topic, start_time, only_last_name, base_name, data)
    dSel = select(bag,'Topic',Topic);
    
    msg_struct = cell2mat(readMessages(dSel,'DataFormat','struct'));

    if(isfield(msg_struct, "Header"))
        tstamps_sec = double(vertcat(vertcat(vertcat(msg_struct.Header).Stamp).Sec));
        tstamps_nsec = double(vertcat(vertcat(vertcat(msg_struct.Header).Stamp).Nsec));
         % Extract time data
        tstamps = tstamps_sec*1e9 + tstamps_nsec;
        tstamps_rel = (tstamps - start_time)/1e9;
    else
        tstamps_rel = [0];
    end

    leading_name = "";
    if(~only_last_name)
        leading_name = strsplit(Topic{1}, "/");
        leading_name = string(leading_name{end});
        % leading_name = string(base_name)+ "__" + string(leading_name{end});
    end


    data = getData(msg_struct, tstamps_rel, leading_name, only_last_name, data);

end
