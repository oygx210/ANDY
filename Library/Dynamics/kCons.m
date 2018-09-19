classdef kCons < kSysObj
    properties(SetAccess=protected)
        Sym;
        Expression;
        DtExpression;
        GFExpression;
        SoftFactor=0;
        
        Jacobian;
        Coriolis;
        GFMatrix=0;
    end
    
    methods(Access=public)
        function obj=kCons(inName,inSys,inMultSym)
            obj@kSysObj(inName,inSys,inSys.Space.RootFrame,inSys.Space.RootFrame);
            if(isa(inMultSym,'char'))
                obj.Sym=sym(inMultSym,'real');
            else
                error(obj.msgStr('Error','Multiplier need to be symbolic variable name'));
            end
            obj.System.Constraint.reg(obj);
        end
        
        function obj=setConsProp(obj,injac,incor,ingf)
            obj.Jacobian=injac;
            obj.Coriolis=incor;
            obj.GFMatrix=ingf;
        end
        
        function [jac,cor,gf]=genConsProp(obj)
            disp(obj.msgStr('Message',['Generating Constraint Force...']));
            Expr=obj.Expression;
            dtExpr=obj.DtExpression;
            gfExpr=obj.GFExpression;
            dtVec=obj.System.getContVector(1);
            sFactor=obj.SoftFactor;
            
            jac=jSimplify(jacobian(Expr,dtVec));
            cor=jacobian(dtExpr,dtVec)*dtVec;
            if(~isempty(obj.InvolvedFrame))
                FrameList=obj.System.Space.getNode(obj.InvolvedFrame);
                FrameVec=sym(zeros(3,numel(obj.InvolvedFrame)));
                dtFrameVec=sym(zeros(6,numel(obj.InvolvedFrame)));
                SubVec=sym(zeros(3,numel(obj.InvolvedFrame)));
                dtSubVec=sym(zeros(6,numel(obj.InvolvedFrame)));
                CorVec=sym(zeros(6,numel(obj.InvolvedFrame)));
                FrameJacobian=sym(zeros(6,numel(dtVec),numel(obj.InvolvedFrame)));
                for ii=1:numel(FrameList)
                    curFrame=FrameList{ii};
                    FrameVec(:,ii)=curFrame.getTransSym(0);
                    dtFrameVec(1:3,ii)=curFrame.getTransSym(1);
                    dtFrameVec(4:6,ii)=curFrame.getAngSym(1);
                    SubVec(:,ii)=curFrame.rootTransDis();
                    dtSubVec(1:3,ii)=curFrame.rootTransVel();
                    dtSubVec(4:6,ii)=curFrame.rootAngVel();
                    CorVec(1:3,ii)=curFrame.rootCorTransAcc();
                    CorVec(4:6,ii)=curFrame.rootCorAngAcc();
                    FrameJacobian(1:3,:,ii)=curFrame.rootTransJacobian();
                    FrameJacobian(4:6,:,ii)=curFrame.rootAngJacobian();
                    jac=jac+jacobian(Expr,dtFrameVec(:,ii))*FrameJacobian(:,:,ii);
                    cor=cor+jacobian(dtExpr,dtFrameVec(:,ii))*dtFrameVec(:,ii)+jacobian(Expr,dtFrameVec(:,ii))*CorVec(:,ii);
                end
            end
            
            gf=sym(0);
            if(numel(sFactor)==1)
                for ii=1:numel(gfExpr)
                    gf=gf+sFactor*(gfExpr(ii));
                end
            elseif(numel(sFactor)==numel(gfExpr))
                for ii=1:numel(gfExpr)
                    gf=gf+sFactor(ii)*(gfExpr(ii));
                end
            else
                error(obj.msgStr('Error','Holonomic Soft Constraint Parameter should be Uniform or Match in Number!'));
            end
            
            obj.CompileInfo.Jacobian=jac;
            obj.CompileInfo.Coriolis=cor;
            obj.CompileInfo.GFFactor=gf;
            
            gf=-jac.'*gf;
            
            TempVec=[];
            if(~isempty(obj.InvolvedFrame))            
                for ii=1:numel(obj.InvolvedFrame)
                    TempVec=[TempVec [FrameVec(:,ii);dtFrameVec(:,ii)]];
                    jac=subs(jac,[FrameVec(:,ii);dtFrameVec(:,ii)],[SubVec(:,ii);dtSubVec(:,ii)]);
                    cor=subs(cor,[FrameVec(:,ii);dtFrameVec(:,ii)],[SubVec(:,ii);dtSubVec(:,ii)]);
                    gf=subs(gf,[FrameVec(:,ii);dtFrameVec(:,ii)],[SubVec(:,ii);dtSubVec(:,ii)]);
                end
            end
            
            obj.CompileInfo.InvolvedFrame=0;
            obj.CompileInfo.SubsVec=sym('SUBSVECTOR__');
            if ~isempty(obj.InvolvedFrame)
                obj.CompileInfo.InvolvedFrame=obj.InvolvedFrame;
                obj.CompileInfo.SubsVec=TempVec;
            end
            
            obj.Jacobian=jac;
            obj.Coriolis=cor;
            obj.GFMatrix=gf;
        end
        
        function obj=setNHCons(obj,inExpr,varargin)
            if isa(inExpr,'sym')&&(numel(inExpr)==1)
                obj.Expression=inExpr;
            else
                error(obj.msgStr('Error','NonHolonomic Constraint Need to be scalar <sym>!'));
            end
            
            if nargin==3
                obj.SoftFactor=varargin{1};
            end
            
            if nargin==4
                obj.GFExpression=[obj.GFExpression;varargin{1}];
                obj.SoftFactor=varargin{2};
            end
            
            obj.setInvolvedFrame(unique([symvar(obj.GFExpression)]));
        end
        
        function obj=setHCons(obj,inExpr,varargin)
            if isa(inExpr,'sym')&&(numel(inExpr)==1)
                obj.Expression=inExpr;
                obj.GFExpression=inExpr;
            else
                error(obj.msgStr('Error','NonHolonomic Constraint Need to be scalar <sym>!'));
            end
            
            [NHSym,NHExpr]=obj.System.getNHSignalVector();
            NHdt=[];
            if ~isempty(NHSym)
                NHdt=pDiff(NHSym,obj.System.TimeVar);
            end
            obj.Expression=jSimplify(subs(pDiff(inExpr,obj.System.TimeVar),NHdt,NHExpr));
            obj.DtExpression=jSimplify(subs(pDiff(obj.Expression,obj.System.TimeVar),NHdt,NHExpr));
            obj.GFExpression=[obj.GFExpression;obj.Expression];
            
            if nargin==3
                obj.SoftFactor=varargin{1};
            end
            
            if nargin==4
                obj.GFExpression=[obj.GFExpression;varargin{1}];
                obj.SoftFactor=varargin{2};
            end
            
            obj.setInvolvedFrame(unique([symvar(obj.GFExpression)]));
        end
    end
end