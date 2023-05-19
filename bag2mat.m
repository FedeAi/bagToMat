%%%%%%%% TEMPLATE %%%%%%%%%%%%%%%
clear all;
close all;

blacklist = ["gazebo","dynamic_reconfigure", "visual"];
whitelist = ["lateral_control","bridge", "odometry"];

%% Load Bag
% Open Bag and select topic
bagToMat(whitelist, blacklist, false)

%% Main function to extract and save data from one or multiple rosbags

function bagToMat(whitelist, blacklist, use_last_name)
    [files,paths] = uigetfile('*.bag', 'Select bag files', 'MultiSelect', 'on');
    file_paths = fullfile(paths, files);
    for i=1:length(file_paths)
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
        
        % Retrive list of topics
        bag_topics = cell(size(bagInfo.Topics));
        % Loop over the struct array and extract the x field from each struct
        for j = 1:numel(bag_topics)
            bag_topics{j} = bagInfo.Topics(j).Topic;
        end
        % Itereate trough al topics and load the messages
        
        for j=1:length(bag_topics)
            if(~contains(bag_topics(j), blacklist) && contains(bag_topics(j), whitelist))
                getMatFromTopic(bag, bag_topics(j), start_time, use_last_name, bag_name)
            end
        end
    end
end


%% Data Extraction Function

function getData( msg_struct, tstamps_rel, base_name, only_last_name)

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
            getData(struct_new{i,1}, tstamps_rel, name_new(i), only_last_name);
            continue;
        
            % Scalar case
        elseif (length(cell2mat(struct_new(i,1))) == 1) 
            temp = cell2mat(struct_new(i,:));
        
            % Vector case
        else
            % prompt = 'Desired vector index:';
            % dlgtitle = name_new(i);
            % dims = [1 35];
            % definput = {'1'};
            % answer = inputdlg(prompt,dlgtitle,dims,definput);
            % index = str2double(answer);
            % index = 1;
            temp = [];
            for j=1:length(struct_new(i,:))
                out = cell2mat(struct_new(i,j));
                if (isstruct(out))
                    % recursive call                
                    getData(out, tstamps_rel, name_new(i), only_last_name);
                    continue;
                end
                % take as temp variable the full vector
                temp = [temp; out'];
            end
            if(length(tstamps_rel) == length(temp))
                var_temp = {tstamps_rel,temp};
            else
                var_temp = [temp];
            end
            x = char(name_new(i));
            assignin('base',x,var_temp)
            continue;
        end
        x = char(name_new(i));
        if(length(tstamps_rel) == length(temp))
            var_temp = [tstamps_rel,temp'];
        else
            var_temp = [temp'];
        end
        var_temp = double(var_temp);
        assignin('base',x,var_temp)
    end
end



%% Main Function for extracting data from a single topic

function getMatFromTopic(bag, Topic, start_time, only_last_name, base_name)
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
        leading_name = string(base_name)+ "__" + string(leading_name{end});
    end


    getData(msg_struct, tstamps_rel, leading_name, only_last_name);

end






