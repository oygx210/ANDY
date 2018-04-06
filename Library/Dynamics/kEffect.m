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
            
            if nargin==2
                simpStyle=string(varargin{1});
            else
                simpStyle="Full";
            end
            
            if(simpStyle=="Full")
                obj.DirVector=jSimplify(obj.DirVector);
                obj.Parameter=jSimplify(obj.Parameter);
                obj.MagVector=jSimplify(obj.MagVector);
            end
            
            dtVec=obj.System.getContVector(1);
            Jacobian=jacobian(obj.DirVector,dtVec);
            if(simpStyle=="Full")||(simpStyle=="Half")
                Jacobian=jSimplify(Jacobian);
            end
            
            if(rem(obj.Power,2)==1)
                Effect=obj.Parameter*obj.MagVector.^obj.Power;
            else
                VectorDir=obj.MagVector./abs(obj.MagVector);
                VectorDir(find(isnan(VectorDir)))=0;
                Effect=obj.Parameter*(VectorDir.*abs(obj.MagVector.^obj.Power));
            end
            
            if(simpStyle=="Full")||(simpStyle=="Half")
                Effect=jSimplify(Effect);
            end
            GF=Jacobian.'*Effect;
            if(simpStyle=="Full")
                GF=jSimplify(GF);
            end
            obj.GFMatrix=GF;
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
        
        function obj=setProperty(obj,inPara,inDir,inMag)
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
        end
    end
end