classdef kLink < jEdge
    properties (SetAccess=protected)
        TransHolo=true; %Holonomic or Nonholonomic Translation
        TransDis=sym([0;0;0]);
        TransVel=sym([0;0;0]);
        TransAcc=sym([0;0;0]);
        
        AngHolo=true; %Holonomic or Nonholonomic Rotation
        RotMat=sym(eye(3,3));
        AngVel=sym([0;0;0]);
        AngAcc=sym([0;0;0]);
        
        TransJacobian;
        AngJacobian;
        CorTransAcc;
        CorAngAcc;
        
    end
    
    properties (SetAccess=protected)
        SimplifyFlag=true;
    end
    
    methods (Access=public)
        function obj=kLink(inName,inFrame1,inFrame2)
            %Construct a kLink Object based on the Rotation and translation frame 
            obj@jEdge(inName,inFrame1,inFrame2);
%             obj.Net.System.Model.setInit(false);
            if(numel(inFrame2.EndEdge.list())>1)
                error(obj.msgStr('Error','The End Frame already has a parent frame!'));
            end
            if(inFrame2.IsRoot)
                error(obj.msgStr('Error','The End Frame is Root, which is invalid!'));
            end
            
        end
        
        function obj=setTransDis(obj,InDis)
            %Set Translational Displacement for Holonomic Link
            if(isa(InDis,'sym')&&(isequal(size(InDis),[3 1])))
                obj.TransDis=InDis;
                obj.TransVel=pDiff(obj.TransDis,obj.Net.TimeVar);
%                 obj.TransAcc=pDiff(obj.TransVel,obj.Net.TimeVar);
            else
                error(obj.msgStr('Error','Input Value should be 3*1 array!'));
            end
            
            obj.TransHolo=true;
            obj.SimplifyFlag=true;
        end
        
        function obj=setTransVel(obj,InVel,inSym)
            %Set Translational Velocity and Initial Displacement for
            %Non-holonomic Link
            if(isa(InVel,'sym')&&(isequal(size(InVel),[3 1])))
                obj.TransDis=sym([0;0;0]);
                obj.TransVel=InVel;
%                 obj.TransAcc=pDiff(obj.TransVel,obj.Net.TimeVar);
            else
                error(obj.msgStr('Error','Input Translational Velocity should be 3*1 arrays!'));
            end
            
            SigArray=sym(strcat(inSym,'_%d'),[3 1]);
            for ii=1:3
                nhSig=obj.Net.System.genNHSignal({char(SigArray(ii)) obj.tagStr(strcat('NHTransDis_',num2str(ii))) 1});
                obj.TransDis(ii)=nhSig.dot(0);
                nhSig.setExpr(InVel(ii));
            end
                
            obj.TransHolo=false;
            obj.SimplifyFlag=true;
        end
        
        function obj=setAngVel(obj,inAngVel,inSym)
            %Set Angular Velocity for and Initial Rotation for
            %Non-holonomic Link
            if(isa(inAngVel,'sym')&&(isequal(size(inAngVel),[3 1])))
                obj.RotMat=sym(eye(3,3));
                obj.AngVel=inAngVel;
%                 obj.AngAcc=pDiff(obj.AngVel,obj.Net.TimeVar);
            else
                error(obj.msgStr('Error','Input Angular Velocity should be 3*1 arrays!'));
            end
            
            SigArray=sym(strcat(inSym,'_%d'),[4 1]);
            sig=cell(4,1);
            for ii=1:4
                nhSig=obj.Net.System.genNHSignal({char(SigArray(ii)) obj.tagStr(strcat('Quat_',num2str(ii))) 1});
                sig{ii}=nhSig;  
            end
            obj.RotMat=quat2Mat([sig{1}.dot(0);sig{2}.dot(0);sig{3}.dot(0);sig{4}.dot(0)]);
            quatVel=0.5*quatMultiply([sig{1}.dot(0);sig{2}.dot(0);sig{3}.dot(0);sig{4}.dot(0)],[0;inAngVel]);
            obj.AngVel=obj.RotMat*inAngVel;
            sig{1}.setExpr(quatVel(1));
            sig{2}.setExpr(quatVel(2));
            sig{3}.setExpr(quatVel(3));
            sig{4}.setExpr(quatVel(4));
            
            obj.AngHolo=false;
            obj.SimplifyFlag=true;
        end
        
        function obj=setYPR(obj,inYPR)
            %Set Rotation based on Yaw Pitch Roll
            if(isa(inYPR,'sym')&&(isequal(size(inYPR),[3 1])))
                yaw=inYPR(3);
                pitch=inYPR(2);
                roll=inYPR(1);
            else
                error(obj.msgStr('Error','Input Value should be [Roll;Pitch;Yaw] (Rotation Notion is Yaw-Pitch-Roll)!'));
            end
            obj.RotMat=ypr2Mat(yaw,pitch,roll).';
            
            AngVelJacobian=[1 0 -sin(pitch);  0 cos(roll) cos(pitch)*sin(roll); 0 -sin(roll) cos(pitch)*cos(roll)];
            obj.AngVel=obj.RotMat*AngVelJacobian*pDiff(inYPR,obj.Net.TimeVar);
%             obj.AngAcc=pDiff(obj.AngVel,obj.Net.TimeVar);
            
            obj.AngHolo=true;
            obj.SimplifyFlag=true;
        end
        
        function obj=setAxang(obj,inAA)
            %Set Rotation based on Axis Angle
            if(isa(inAA,'sym')&&(isequal(size(inAA),[4 1])))
                quat=cos(inAA(1)/2);
                quat=[quat;sin(inAA(1)/2)*inAA(2:4)/(sum(inAA(2:4).^2)^0.5)];
                obj.setQuat(quat);
            else
                error(obj.msgStr('Error','Input Value should be [Angle;X;Y;Z]'));
            end
        end
        
        function obj=setQuat(obj,inQuat)
            %Set Rotation based on Quaternion
            if(isa(inQuat,'sym')&&(isequal(size(inQuat),[4 1])))
                qw=inQuat(1);
                qx=inQuat(2);
                qy=inQuat(3);
                qz=inQuat(4);
            else
                error(obj.msgStr('Error','Input Value should be [qw;qx;qy;qz]'));
            end
            
            obj.RotMat=quat2Mat(inQuat);

            AngVelJacobian=2*[-qx qw -qz qy; -qy qz qw -qx; -qz -qy qx qw];
            dq=pDiff(inQuat,obj.Net.TimeVar);
            obj.AngVel=AngVelJacobian*dq;
%             obj.AngAcc=pDiff(obj.AngVel,obj.Net.TimeVar);

            obj.AngHolo=true;
            obj.SimplifyFlag=true;
        end
        
        function obj=setDH(obj,inDH)
            %Set Kinematic Link as Classic DH Link
            if(isa(inDH,'sym')&&(isequal(size(inDH),[4 1])))
                d=inDH(1);
                theta=inDH(2);
                a=inDH(3);
                alpha=inDH(4);
            else
                error(obj.msgStr('Error','Input Value should be [d;theta;a;alpha] (classic DH parameter)'));
            end
                
            obj.RotMat=...
            [...
            cos(theta), -sin(theta)*cos(alpha), sin(alpha)*sin(theta);...
            sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta);...
            0, sin(alpha), cos(alpha),...
            ];
            obj.AngVel=[pDiff(alpha,obj.Net.TimeVar)*cos(theta);...
                        pDiff(alpha,obj.Net.TimeVar)*sin(theta);...
                        pDiff(theta,obj.Net.TimeVar)];
%             obj.AngAcc=pDiff(obj.AngVel,obj.Net.TimeVar);


            obj.TransDis=[a*cos(theta);a*sin(theta);d];
            obj.TransVel=pDiff(obj.TransDis,obj.Net.TimeVar);
%             obj.TransAcc=pDiff(obj.TransVel,obj.Net.TimeVar);

            obj.TransHolo=true;
            obj.AngHolo=true;
            obj.SimplifyFlag=true;
        end
        
        function obj=genProp(obj,varargin)
            %Simplify the Property Expressions
            t=obj.Net.TimeVar;
            qddt=obj.Net.System.getContVector(2);
            
            obj.TransDis=jSimplify(obj.TransDis);
            obj.TransVel=jSimplify(obj.TransVel);
            obj.RotMat=jSimplify(obj.RotMat);
            obj.AngVel=jSimplify(obj.AngVel);

            obj.TransAcc=jSimplify(pDiff(obj.TransVel,t));
            obj.AngAcc=jSimplify(pDiff(obj.AngVel,t));
            
            [NHSym,NHExpr]=obj.Net.System.getNHSignalVector();
            NHdt=[];
            if ~isempty(NHSym)
                NHdt=pDiff(NHSym,obj.Net.System.TimeVar);
            end
            obj.TransAcc=jSimplify(subs(obj.TransAcc,NHdt,NHExpr));
            obj.AngAcc=jSimplify(subs(obj.AngAcc,NHdt,NHExpr));

            obj.TransJacobian=jSimplify(jacobian(obj.TransAcc,qddt));
            obj.AngJacobian=jSimplify(jacobian(obj.AngAcc,qddt));
            
            obj.CorTransAcc=jSimplify(obj.TransAcc-obj.TransJacobian*qddt);
            obj.CorAngAcc=jSimplify(obj.AngAcc-obj.AngJacobian*qddt);
            
            obj.SimplifyFlag=false;
        end
        
        function output=FTF(obj)
            %Coordinate Transformation from End Coordinate Systsem to
            %Start Coordinate System
            output=[obj.RotMat,obj.TransDis;zeros(1,3),1];
        end
        
        function output=BTF(obj)
            %Coordinate Transformation from Start Coordinate Systsem to
            %End Coordinate System
            output=[obj.RotMat.',-(obj.RotMat.')*obj.TransDis;zeros(1,3),1];
        end
    end
end
