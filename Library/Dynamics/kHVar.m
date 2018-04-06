classdef kHVar < jObj & sym
    properties(SetAccess=protected)
        System;
    end
    
    methods(Access=public)
        function obj=kHVar(inName,inSys,inDes)
            obj@jObj(inDes);
            obj@sym(inName,'real');
            
            if(isa(inSys,'kSystem'))
                obj.System=inSys;
                obj.System.Model.setInit(false);
            else
                error(obj.msgStr('Error','Link for Coordinate Induction should be <kLink>!'));
            end
        end
    end
end