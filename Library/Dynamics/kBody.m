classdef kBody < kSysObj
    properties(SetAccess=protected)
        Mass=0;
        Moment=zeros(3,3);
        
        Force;
        Torque;
        
        GFMatrix=0;
        InertMatrix=0;
    end
    
    methods(Access=public)
        function obj=kBody(inName,inSys,inFrame)
            obj@kSysObj(inName,inSys,inFrame,inFrame);
            obj.System.Body.reg(obj);
            
            obj.Force=jDic(obj.tagStr('Force'),'kForce');
            obj.Torque=jDic(obj.tagStr('Torque'),'kTorque');
        end
        
        
        function varargout=genForce(obj,inForceList)
            if ~(isa(inForceList,'cell'))
                error(obj.msgStr('Error','Input list of body should be cell array!'));
            end
            
            ListSize=size(inForceList);
            if ListSize(2)~=3
                error(obj.msgStr('Error','Input list of body should follow format of {name ActionFrame DirectionRefFrame;...}!'));
            end
            
            varargout=cell(ListSize(1),1);
            for ii=1:ListSize(1)                
                varargout{ii}=kForce(inForceList{ii,1},obj,inForceList{ii,2},inForceList{ii,3});
            end
        end
        
        function varargout=genTorque(obj,inTorqueList)
            if ~(isa(inTorqueList,'cell'))
                error(obj.msgStr('Error','Input list of body should be cell array!'));
            end

            ListSize=size(inTorqueList);
            if ListSize(2)~=3
                error(obj.msgStr('Error','Input list of body should follow format of {name ActionFrame DirectionRefFrame;...}!'));
            end

            varargout=cell(ListSize(1),1);
            for ii=1:ListSize(1)                
                varargout{ii}=kTorque(inTorqueList{ii,1},obj,inTorqueList{ii,2},inTorqueList{ii,3});
            end
        end
        
        function obj=setProp(obj,varargin)
            if nargin==2
                inMass=varargin{1};
            elseif nargin==3
                inMass=varargin{1};
                inMoment=varargin{2};
            else
                error(obj.msgStr('Error','Input should be Mass and Moment(optional)!'));
            end
                
            if (isa(inMass,'sym')&&(numel(inMass)==1))
                if (isequal(obj.Mass,sym(0)))
                    error(obj.msgStr('Error','Mass cannot be zero!'));
                else
                    obj.Mass=inMass;
                end
            else
                error(obj.msgStr('Error','The Mass need to be a 1*1 sym!'));
            end
            
            if nargin==3
                if (isa(inMoment,'sym')&&(isequal(size(inMoment),[3,3])))
                    obj.Moment=inMoment;
                else
                    error(obj.msgStr('Error','The Moment need to be a 3*3 sym!'));
                end
            end
        end
        
        function obj=setInertia(obj,inM,inGF)
            obj.InertMatrix=inM;
            obj.GFMatrix=inGF;
        end
        
        function [MMat,FMat]=genInertia(obj,varargin)
            disp(obj.msgStr('Message',['Generating Inertia Matrix and Coriolis Force...']));
            
            dtVec=obj.System.getContVector(1);
            ddtVec=obj.System.getContVector(2);
            
            if nargin==2
                simpStyle=string(varargin{1});
            else
                simpStyle='Full';
            end
            
            rotor=obj.BaseFrame.rootRotMat();
            if(simpStyle=="Full")||(simpStyle=="Half")
                rotor=jSimplify(rotor);
            end
            
            m=obj.Mass;
            I=rotor*obj.Moment*rotor.';
            v=obj.BaseFrame.rootTransVel;
            w=obj.BaseFrame.rootAngVel;
            a=obj.BaseFrame.rootTransAcc;
            alpha=obj.BaseFrame.rootAngAcc;
            
            if(simpStyle=="Full")||(simpStyle=="Half")
                m=jSimplify(m);
                I=jSimplify(I);
                v=jSimplify(v);
                w=jSimplify(w);
                a=jSimplify(a);
                alpha=jSimplify(alpha);
            end
            
            if(~isa(dtVec,'sym'))
                error(obj.msgStr('Error','Generalized Velocity Vector need to be sym!'));
            end
            if(~isa(ddtVec,'sym'))
                error(obj.msgStr('Error','Generalized Velocity Vector need to be sym!'));
            end
            
            dtVec=reshape(dtVec,[],1);
            ddtVec=reshape(ddtVec,[],1);
            
            vJacobian=jacobian(v,dtVec);
            wJacobian=jacobian(w,dtVec);
            
            if(simpStyle=="Full")||(simpStyle=="Half")
                vJacobian=jSimplify(vJacobian);
                wJacobian=jSimplify(wJacobian);
            end
            
            MMat=vJacobian.'*m*vJacobian+wJacobian.'*I*wJacobian;
            
            if(simpStyle=="Full")
                MMat=jSimplify(MMat);
            end
            
%             FMat=MMat*ddtVec-vJacobian.'*m*a-wJacobian.'*(I*alpha+cross(w,I*w));
            FMat=subs(-vJacobian.'*m*a-wJacobian.'*(I*alpha+cross(w,I*w)),ddtVec,zeros(numel(ddtVec),1));
            
            if(simpStyle=="Full")
                FMat=jSimplify(FMat);
            end
            
            obj.InertMatrix=MMat;
            obj.GFMatrix=FMat;
        end
        
        function output=genKineticEnergy(obj)
            m=obj.Mass;
            I=obj.Moment;
            v=obj.BaseFrame.rootTransVel;
            w=obj.BaseFrame.rootAngVel;
            
            output=0.5*((v.')*m*v+(w.')*I*w);
        end
    end
end