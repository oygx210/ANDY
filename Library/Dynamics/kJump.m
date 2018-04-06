classdef kJump < jEdge
    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=kJump(inName,inFlow1,inFlow2)
            obj@jEdge(inName,inFlow1,inFlow2);
        end
        
        function obj=makeJump(obj,inExpr)
            ui=input(obj.msgStr('Warning','Making new jump file will erase existing jump file data, do you want to continue?(''Y'' for YES, anything else for NO)'),'s');
            if(~strcmp(ui,'Y'))
                return;
            end
            if(~obj.Net.IsInitialized)
                obj.Net.Init();
            end
            fileDir=obj.Net.Directory;
            cd(fileDir);
            paramSym=obj.Net.Parameter(:,1);
            paramVal=obj.Net.Parameter(:,2);
            funcName=obj.jumpTagStr([]);
            funcid=fopen(strcat(funcName,'.m'),'w');
            tmpContent=fReadLine('jumpTemplate.tm');
            tmpContent=strrep(tmpContent,'FUNCNAME_',obj.jumpTagStr([]));
            tmpContent=strrep(tmpContent,'FROMFLOW_',num2str(obj.EdgeNumber(1)));
            tmpContent=strrep(tmpContent,'TOFLOW_',num2str(obj.EdgeNumber(2)));
            fWriteLine(funcid,tmpContent);
            
            act=sym('ACT');
            flow=sym('FLOW');
            count=sym('COUNT');
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
            
            if((isa(inExpr,'cell'))&&(~isempty(inExpr)))
                exprSize=size(inExpr);
                if(exprSize(2)==2)
                    if isempty(obj.EndNode.EOM.ConsVector)
                        consExpr=zeros(numel(q),1);
                    else
                        consExpr=obj.EndNode.EOM.ConsJacobian;
                    end
                    inExpr=[inExpr;{'JCons' consExpr}];
                    for ii=1:exprSize(1)+1
                        realExpr=jSimplify(subs(inExpr{ii,2},paramSym,paramVal));
                        matlabFunction(realExpr,'File',strcat(inExpr{ii,1},'.mx'),'Vars',{act,flow,count,t,q,p,u,s,l});
                        fWriteLine(funcid,[fReadLine(strcat(inExpr{ii,1},'.mx'));'end'],1);
                    end
                else
                    error(obj.msgStr('Error','The Input should be a cell with format {<''name''> <sym>;...}!'));
                end
            end
            fWriteLine(funcid,{'end'});
            delete('*.mx');
            fclose(funcid);
            
            cd('..');
        end
        
        function obj=editJump(obj)
            fileDir=obj.Net.Directory;
            cd(fileDir);
            funcName=obj.jumpTagStr([]);
            edit(strcat(funcName,'.m'));
            cd('..');
        end
        
        function output=jumpTagStr(obj,inStr)
            output=obj.Net.System.tagStr(strcat(inStr,'Jump_',num2str(obj.EdgeNumber(1)),'_',num2str(obj.EdgeNumber(2))));
        end
    end
end