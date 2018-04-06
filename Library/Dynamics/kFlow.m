classdef kFlow < jNode
    properties(SetAccess=protected)
        Constraint=[];
        
        EOM=[];
    end
    
    methods(Access=public)
        function obj=kFlow(inName,inNet)
            obj@jNode(inName,inNet);
        end
        
        function obj=genFlowEOM(obj,inCons)
            obj.Constraint=inCons;
            if(~obj.Net.IsInitialized)
                obj.Net.Init();
            end
            obj.EOM.InertMatrix=obj.Net.BaseEOM.InertMatrix;
            obj.EOM.GFMatrix=obj.Net.BaseEOM.GFMatrix;
            ConsEOM=obj.Net.System.getConsSet(inCons);
            obj.EOM.ConsVector=ConsEOM.ConsVector;
            obj.EOM.ConsJacobian=ConsEOM.ConsJacobian;
            obj.EOM.ConsCoriolis=ConsEOM.ConsCoriolis;
            obj.EOM.ConsGFMatrix=ConsEOM.ConsGFMatrix;          
        end
        
        function obj=makeFlow(obj)
            if(isempty(obj.EOM))
                error(obj.msgStr('Error','Please generate flow EOM before make flow files!'))
            end
            fileDir=obj.Net.Directory;
            cd(fileDir);
            paramSym=obj.Net.Parameter(:,1);
            paramVal=obj.Net.Parameter(:,2);
            if(~isempty(obj.EOM.ConsVector))
                consNum=sym(zeros(numel(obj.Constraint),1));
                for ii=1:numel(obj.Constraint)
                    consNum(ii)=sym(find(ismember(obj.Net.System.Constraint.Content,obj.Constraint{ii})));
                end
                ConsGFMatrix=0;
                cgf=obj.EOM.ConsGFMatrix;
                parfor ii=1:numel(cgf(1,:))
                    ConsGFMatrix=ConsGFMatrix+jSimplify(subs(cgf(:,ii),paramSym,paramVal));
                end
                parfor ii=1:numel(ConsGFMatrix)
                    ConsGFMatrix(ii,1)=jSimplify(ConsGFMatrix(ii,1));
                end
%                 ConsGFMatrix=(sum(subs([obj.EOM.ConsGFMatrix],paramSym,paramVal),2));
                JCons=(subs(obj.EOM.ConsJacobian,paramSym,paramVal));
                CorMatrix=(subs(obj.EOM.ConsCoriolis,paramSym,paramVal));
            else
                consNum=sym(0);
                ConsGFMatrix=sym(0);%0*GFMatrix;
                JCons=sym(0);
                CorMatrix=sym(0);
            end
            
            t=obj.Net.System.TimeVar;
            q=obj.Net.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Net.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Net.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.Net.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.Net.NHSignal(:,1);
            end
            l=obj.Net.BaseCons.ConsVector;
            if isempty(l)
                l=sym('CONSSYM');
            end
                        
            matlabFunction(consNum,'File','activeConsNum.mx');
            matlabFunction(JCons,'File','partialConsJacobian.mx','Vars',{t,q,p,u,s,l});
            matlabFunction(CorMatrix,'File','consCorMat.mx','Vars',{t,q,p,u,s,l});
            matlabFunction(ConsGFMatrix,'File','consGFMat.mx','Vars',{t,q,p,u,s,l});
            
            funcName=obj.flowTagStr([]);
            Content=fReadLine('flowTemplate.tm');
            Content=strrep(Content,'FUNCNAME_',funcName);
            funcid=fopen(strcat(funcName,'.m'),'w');
            fWriteLine(funcid,Content);
            fWriteLine(funcid,[fReadLine('activeConsNum.mx');'end'],1);
            fWriteLine(funcid,[fReadLine('partialConsJacobian.mx');'end'],1);
            fWriteLine(funcid,[fReadLine('consCorMat.mx');'end'],1);
            fWriteLine(funcid,[fReadLine('consGFMat.mx');'end'],1);
            fWriteLine(funcid,{'end'});
            delete('*.mx');
            fclose(funcid);
            
            cd('..');
        end
        
        function output=flowTagStr(obj,inStr)
            output=obj.Net.System.tagStr(strcat(inStr,'Flow_',num2str(obj.NodeNumber)));
        end
    end
end