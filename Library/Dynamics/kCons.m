classdef kCons < kSysObj
    properties(SetAccess=protected)
        IsHolonomic=false;
        
        Sym;
        Expression;
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
            inExpr=obj.Expression;
            dtVec=obj.System.getContVector(1);
            ddtVec=obj.System.getContVector(2);
            eta=obj.SoftFactor;
            [NHSym,NHExpr]=obj.System.getNHSignalVector();
            NHdt=[];
            if ~isempty(NHSym)
                NHdt=pDiff(NHSym,obj.System.TimeVar);
            end
            
            if obj.IsHolonomic
                dtExpr=subs(pDiff(inExpr,obj.System.TimeVar),NHdt,NHExpr);
                jac=jSimplify(jacobian(dtExpr,dtVec));
                ddtExpr=subs(pDiff(dtExpr,obj.System.TimeVar),NHdt,NHExpr);
%                 cor=jSimplify(ddtExpr-jac*ddtVec);
                cor=jSimplify(subs(ddtExpr,ddtVec,zeros(numel(ddtVec),1)));
                if(numel(eta)==1)
                    gf=(-jac.'*(inExpr+dtExpr))*eta;
                elseif(numel(eta)==2)
                    gf=(-jac.'*(inExpr*eta(1)+dtExpr*eta(2)));
                else
                    error(obj.msgStr('Error','Holonomic Soft Constraint Requries no more than 2 parameters!'));
                end
            else
                jac=jSimplify(jacobian(inExpr,dtVec));
                dtExpr=subs(pDiff(inExpr,obj.System.TimeVar),NHdt,NHExpr);
%                 cor=jSimplify(dtExpr-jac*ddtVec);
                cor=jSimplify(subs(dtExpr,ddtVec,zeros(numel(ddtVec),1)));
                if(numel(eta)==1)
                    gf=(-jac.'*(inExpr))*eta;
                else
                    error(obj.msgStr('Error','Nonholonomic Soft Constraint Requries no more than 1 parameter!'));
                end
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
            
            obj.IsHolonomic=false;
        end
        
        function obj=setHCons(obj,inExpr,varargin)
            if isa(inExpr,'sym')&&(numel(inExpr)==1)
                obj.Expression=inExpr;
            else
                error(obj.msgStr('Error','NonHolonomic Constraint Need to be scalar <sym>!'));
            end
            
            if nargin==3
                obj.SoftFactor=varargin{1};
            end
            
            obj.IsHolonomic=true;
        end
    end
end