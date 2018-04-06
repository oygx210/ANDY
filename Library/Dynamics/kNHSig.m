classdef kNHSig < kNHVar
    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=kNHSig(inName,inSys,inDes,inOrder)
            obj@kNHVar(inName,inSys,inDes,inOrder);
            inSys.NHSignal.reg(obj);
        end
    end
end