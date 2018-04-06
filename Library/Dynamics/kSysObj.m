classdef kSysObj < jObj
    %Element in the System which are established based on the Coordinate
    %Frames
    properties(SetAccess=protected)
        System;
        TimeVar;
        
        BaseFrame;
        ReferenceFrame;
    end
    
    methods(Access=public)
        function obj=kSysObj(inName,inSys,inBaseFrame,inRefFrame)
            obj@jObj(inName);
            if isa(inSys,'kSystem')
                obj.System=inSys;
                obj.TimeVar=obj.System.TimeVar;
                obj.System.Model.setInit(false);
            else
                error(obj.msgStr('Error','The Second Input Need to be kSys!'));
            end
            
            if isa(inBaseFrame,'kFrame')
                obj.BaseFrame=inBaseFrame;
            elseif ischar(inBaseFrame)
                obj.BaseFrame=obj.System.Space.Node.get(inBaseFrame);
            else
                error(obj.msgStr('Error','The Second Input Need to be <kFrame> or the NameTag of <kFrame>!'));
            end
            
            if isa(inRefFrame,'kFrame')
                obj.ReferenceFrame=inRefFrame;
            elseif ischar(inRefFrame)
                obj.ReferenceFrame=obj.System.Space.Node.get(inRefFrame);
            else
                error(obj.msgStr('Error','The Third Input Need to be <kFrame> or the NameTag of <kFrame>!'));
            end
        end
    end
end