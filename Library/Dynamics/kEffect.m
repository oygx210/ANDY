classdef kEffect < kSysObj
    properties(SetAccess=protected)
        %Yielding:
        %GF=jacobian(DirVector,dq).'*(Parameter)*(MagVector./abs(MagVector.^(Power-1)))
        
        Dimension=0;      %The Dimension of Effect (m)
        Power=0;          %The Power of Effect (a)
        
        Parameter=0;      %The Parameter Matrix m*m (P)
        DirVector=0;      %The Direction Vector m*1 (Vd)
        MagVector=0;      %The Magnitude Vector m*1 (Vm)
        
        GFMatrix=0;
    end
    
    methods(Access=public)
        function obj=kEffect(inName,inSys,inBaseFrame,inDirFrame)
            obj@kSysObj(inName,inSys,inBaseFrame,inDirFrame);
        end
        
        function obj=setGF(obj,InGF)
            obj.GFMatrix=InGF;
        end
        
        function GF=genGF(obj,varargin)
            disp(obj.msgStr('Message',['Generating Generalized Force...']));
            obj.DirVector=jSimplify(obj.DirVector);
            obj.Parameter=jSimplify(obj.Parameter);
            obj.MagVector=jSimplify(obj.MagVector);
            
            dtVec=obj.System.getContVector(1);
            if isa(obj,'kForce')
%                 Jacobian=obj.BaseFrame.rootTransJacobian();
                  Jacobian=0;
            elseif isa(obj,'kTorque')
%                 Jacobian=obj.BaseFrame.rootAngJacobian();
                  Jacobian=0;
            else
                Jacobian=jacobian(obj.DirVector,dtVec);
                Jacobian=jSimplify(Jacobian);
            end
            
            if(~isempty(obj.InvolvedFrame))
                FrameList=obj.System.Space.getNode(obj.InvolvedFrame);
                FrameVec=sym(zeros(3,numel(obj.InvolvedFrame)));
                dtFrameVec=sym(zeros(6,numel(obj.InvolvedFrame)));
                SubVec=sym(zeros(3,numel(obj.InvolvedFrame)));
                dtSubVec=sym(zeros(6,numel(obj.InvolvedFrame)));
                FrameJacobian=sym(zeros(6,numel(dtVec),numel(obj.InvolvedFrame)));
                for ii=1:numel(FrameList)
                    curFrame=FrameList{ii};
                    FrameVec(:,ii)=curFrame.getTransSym(0);
                    dtFrameVec(1:3,ii)=curFrame.getTransSym(1);
                    dtFrameVec(4:6,ii)=curFrame.getAngSym(1);
                    SubVec(:,ii)=curFrame.rootTransDis();
                    dtSubVec(1:3,ii)=curFrame.rootTransVel();
                    dtSubVec(4:6,ii)=curFrame.rootAngVel();
                    FrameJacobian(1:3,:,ii)=curFrame.rootTransJacobian();
                    FrameJacobian(4:6,:,ii)=curFrame.rootAngJacobian();
                    Jacobian=Jacobian+jacobian(obj.DirVector,dtFrameVec(:,ii))*FrameJacobian(:,:,ii);
                end
            end
            if(rem(obj.Power,2)==1)
                Effect=obj.Parameter*obj.MagVector.^obj.Power;
            else
                VectorDir=obj.MagVector./abs(obj.MagVector);
                VectorDir(find(isnan(VectorDir)))=0;
                Effect=obj.Parameter*(VectorDir.*abs(obj.MagVector.^obj.Power));
            end
            
            obj.CompileInfo.ReferenceFrame=0;
            obj.CompileInfo.ActionFrame=0;
            obj.CompileInfo.Effect=Effect;
            
            if isa(obj,'kForce')||isa(obj,'kTorque')
                Effect=obj.ReferenceFrame.rootRotMat*Effect;
                obj.CompileInfo.ReferenceFrame=[obj.ReferenceFrame.NodeNumber];
                if isa(obj,'kForce')
                    obj.CompileInfo.ActionFrame=[obj.BaseFrame.NodeNumber];%Indicates Translational Jacobian;
                else
                    obj.CompileInfo.ActionFrame=-[obj.BaseFrame.NodeNumber];%Indicates Angular Jacobian;
                end
            end
            
            GF=Jacobian.'*Effect;
            %Important Note: Control input symbolic variables should not 
            %be concealed in any intermediate factors, appear in direction
            %vector or appear with a order higher than 2, otherwise they
            %will be ignored as normal forces.
            
            inputJacobian=sym(0);
            if ~isempty(obj.System.getInputVector())
                inputJacobian=jacobian(Effect,obj.System.getInputVector());
            end
            
            TempVec=[];
            if(~isempty(obj.InvolvedFrame))            
                for ii=1:numel(obj.InvolvedFrame)
                    TempVec=[TempVec [FrameVec(:,ii);dtFrameVec(:,ii)]];
                    GF=subs(GF,[FrameVec(:,ii);dtFrameVec(:,ii)],[SubVec(:,ii);dtSubVec(:,ii)]);
                end
            end
            obj.GFMatrix=GF;
            
            %Compile Info Collection
            obj.CompileInfo.InvolvedFrame=0;
            obj.CompileInfo.SubsVec=sym('SUBSVECTOR__');
            if ~isempty(obj.InvolvedFrame)
                obj.CompileInfo.InvolvedFrame=obj.InvolvedFrame;
                obj.CompileInfo.SubsVec=TempVec;
            end
            obj.CompileInfo.Jacobian=Jacobian;
            obj.CompileInfo.InputJacobian=inputJacobian;
        end
    end
    
    methods(Access=protected)
        function obj=setType(obj,inDim,inPower)
            if inDim>=1
                obj.Dimension=floor(inDim);
            else
                error(obj.msgStr('Error','Dimension must be positive integer!'));
            end
            
            if inPower>=0
                obj.Power=floor(inPower);
            else
                error(obj.msgStr('Error','Power must be positive integer!'));
            end
        end
        
        function obj=setProperty(obj,inDir,inPara,inMag)
            paraSize=size(inPara);
            dirSize=size(inDir);
            magSize=size(inMag);
            if isequal(paraSize,[obj.Dimension,obj.Dimension])&&isa(inPara,'sym')
                obj.Parameter=inPara;
            else
                error(obj.msgStr('Error','Parameter Size does not match effect dimension or is not <sym>!'));
            end
            
            if isequal(dirSize,[obj.Dimension,1])&&isa(inDir,'sym')
                obj.DirVector=inDir;
            else
                error(obj.msgStr('Error','DirVector Size does not match dimension or is not <sym>!'));
            end
            
            if isequal(magSize,[obj.Dimension,1])&&isa(inMag,'sym')
                obj.MagVector=inMag;
            else
                error(obj.msgStr('Error','MagVector Size does not match dimension or is not <sym>!'));
            end
            
            obj.setInvolvedFrame(unique([symvar(inPara),symvar(inDir),symvar(inMag)]));
        end
    end
end