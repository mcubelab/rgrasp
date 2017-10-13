function message = parseMessage(line,mode)
        if strcmp(mode,'offline')
            alldata = strsplit(line);
            message.binname = alldata{1};
            message.imagesData{1} = alldata(2:4);
            message.imagesData{2} = alldata(5:7);
            message.mode = alldata{8};
            message.objectName = lower(alldata{9});
        elseif strcmp(mode,'debug')
            alldata = strsplit(line);
            message.binname = alldata{2};
            message.mode = alldata{3};
            message.objectName = lower(alldata{4});
            message.imagesData{1} = {alldata{1}, message.binname,'0'};
            message.imagesData{2} = {alldata{1}, message.binname,'1'};
            if length(alldata) == 7
                message.dropxyz = str2double(alldata(5:7));
            end
        else
            alldata = strsplit(line);
            message.binname = alldata{1};
            message.mode = alldata{2};
            if length(alldata)>2
                message.objectName = lower(alldata{3});
            else
                message.objectName = '-';
            end
            message.imagesData{1} = {'/', message.binname,'0'};
            message.imagesData{2} = {'/', message.binname,'1'};
            if length(alldata) == 6
               message.dropxyz = str2double(alldata(4:6));
            end
        end
end