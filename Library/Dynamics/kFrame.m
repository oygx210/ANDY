classdef kFrame < jNode
    properties(SetAccess=protected)
        IsRoot=false;
        RootChain=[];
    end
    
    methods(Access=public)
        function obj=kFrame(inName,inNet)
            obj@jNode(inName,inNet);
            obj.Net.System.Model.setInit(false);
        end
        
        function obj=setRoot(obj,inBool)
            obj.IsRoot=inBool;
        end
        
        function output=rootChain(obj)
            output=obj.NodeNumber;
            if obj.IsRoot
                return;
            end
            
            chain=obj.Net.path(obj.Net.RootFrame,obj);
            output=output*ones(1,numel(chain)+1);
            for ii=numel(chain):-1:1
                output(1,ii)=chain{ii}.EdgeNumber(1);
            end
        end
        
        function output=rootTF(obj)
            output=sym(eye(4,4));
            if(obj.IsRoot)
                return;
            end
            startChain=obj.Net.path(obj.Net.RootFrame,obj);
            for ii=numel(startChain):-1:1
                output=(startChain{ii}.FTF())*output;
            end
        end
        
        function output=fwdTF(obj,inStart)
            if(~(ischar(inStart)||isa(inStart,'kFrame')))
                error(obj.msgStr('Error','Input should be char or kFrame!'));
            end
            
            startChain=obj.Net.path(inStart,obj);
            if isempty(startChain)
                warning(obj.msgStr('Warning','There is no derivation path from the Input Frame!'));
            end
            output=eye(4,4);
            for ii=numel(startChain):-1:1
                output=(startChain{ii}.FTF())*output;
            end
        end
        
        function output=bwdTF(obj,inEnd)
            if(~(ischar(inEnd)||isa(inEnd,'kFrame')))
                error(obj.msgStr('Error','Input should be char or kFrame!'));
            end
            
            endChain=obj.Net.path(inEnd,obj);
            if isempty(endChain)
                warning(obj.msgStr('Error','There is no derivation path from the Input Frame!'));
            end
            output=eye(4,4);
            for ii=1:1:numel(endChain)
                output=(endChain{ii}.BTF())*output;
            end
        end
        
        function output=rootTransDis(obj)
            rootTFMat=obj.rootTF();
            output=rootTFMat(1:3,4);
        end
        
        function output=rootTransVel(obj)
            output=zeros(3,1);
            tfChain=eye(4,4);
            linkChain=obj.Net.path(obj.Net.RootFrame,obj);
            if isempty(linkChain)
                return
            else
                chainNum=numel(linkChain);
                for ii=chainNum:-1:1
                    rotor=linkChain{ii}.RotMat;
                    disChain=rotor*tfChain(1:3,4);
                    output= linkChain{ii}.TransVel...
                            +rotor*output...
                            +cross(linkChain{ii}.AngVel,disChain);
                    tfChain=linkChain{ii}.FTF()*tfChain;
                end
            end
        end
        
        function output=rootTransAcc(obj)
            output=zeros(3,1);
            vel=zeros(3,1);
            tfChain=eye(4,4);
            linkChain=obj.Net.path(obj.Net.RootFrame,obj);
            if isempty(linkChain)
                return
            else
                chainNum=numel(linkChain);
                for ii=chainNum:-1:1
                    rotor=linkChain{ii}.RotMat;
                    disChain=rotor*tfChain(1:3,4);
                    output= linkChain{ii}.TransAcc...
                            +cross(linkChain{ii}.AngAcc,disChain)...
                            +rotor*output...
                            +2*cross(linkChain{ii}.AngVel,rotor*vel)...
                            +cross(linkChain{ii}.AngVel,cross(linkChain{ii}.AngVel,disChain));
                    vel=    linkChain{ii}.TransVel...
                            +rotor*vel...
                            +cross(linkChain{ii}.AngVel,disChain);
                    tfChain=linkChain{ii}.FTF()*tfChain;
                end
            end
        end
        
        function output=rootRotMat(obj)
            rootTFMat=obj.rootTF();
            output=rootTFMat(1:3,1:3);
        end
        
        function output=rootAngVel(obj)
            output=zeros(3,1);
            linkChain=obj.Net.path(obj.Net.RootFrame,obj);
            if isempty(linkChain)
                return
            else
                chainNum=numel(linkChain);
                for ii=chainNum:-1:1
                    output=linkChain{ii}.AngVel+linkChain{ii}.RotMat*output;
                end
            end
        end
        
        function output=rootAngAcc(obj)
            output=zeros(3,1);
            vel=zeros(3,1);
            linkChain=obj.Net.path(obj.Net.RootFrame,obj);
            if isempty(linkChain)
                return
            else
                chainNum=numel(linkChain);
                for ii=chainNum:-1:1
                    output= linkChain{ii}.AngAcc...
                            +linkChain{ii}.RotMat*output...
                            +cross(linkChain{ii}.AngVel,linkChain{ii}.RotMat*vel);
                    vel=linkChain{ii}.AngVel+linkChain{ii}.RotMat*vel;
                end
            end
        end
    end
end