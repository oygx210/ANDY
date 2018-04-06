function Content=fReadLine(fileName)
% Content=fReadLine(fileName)
% A wrapper function to read file text lines into a cell output from the
% name of the file <fileName> (make sure that the file exists in the
% current path!).

% Copyright 2018 Robotics & Mechatronics Lab, Virginia Tech.
% Author: Jiamin Wang; Revised: 18-Mar-2018 01:23:48

    Content={};
    id=fopen(fileName,'r');
    while(1)
        Content=cat(1,Content,fgetl(id));
        if(Content{end}==-1)
            Content=reshape({Content{1:end-1}},[],1); 
            break;
        end
    end
    fclose(id);
end
