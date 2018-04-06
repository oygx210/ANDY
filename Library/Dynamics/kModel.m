classdef kModel < jNet
    properties(SetAccess=protected)
        System;
        Directory;
        IsPrecompile=true;
        IsInitialized=false;
        
        Parameter;
        Continuous;
        Discrete;
        Input;
        NHSignal;
        
        BaseEOM;
        BaseCons;
        
        CountLimit=100;
    end
    
    methods(Access=public)
        function obj=kModel(inName,inSys)
            obj@jNet(inName,'kFlow','kJump');
            obj.System=inSys;
            obj.setDir(true);
            obj.Directory=strcat(obj.System.tagStr('Dynamics'));
            if(~exist(obj.Directory))
                mkdir(obj.Directory);
            end
            cd(obj.Directory);
            obj.Directory=pwd;
            disp(obj.msgStr('Notification',strcat('System Dynamics will be saved in ./',(obj.System.tagStr('Model')))));
            disp(obj.msgStr('Notification','Please save the <jump> function in this directory'));
            cd('..');
        end
        
        function obj=setInit(obj,inBool)
            obj.IsInitialized=inBool;
        end
        
        function obj=setCountLimit(obj,inNum)
            if(inNum<1)
                error(obj.msgStr('Error','Count Limit should be positive Integer!'));
            end
            obj.CountLimit=floor(abs(inNum));
        end
        
        function obj=Init(obj)
            [param,val]=obj.System.getParamVector();
            obj.Parameter=[param,val];
            obj.Discrete=obj.System.getDiscVector();
            obj.Input=obj.System.getInputVector();
            obj.Continuous=[obj.System.getContVector(0);obj.System.getContVector(1)];
            [Sym,Val]=obj.System.getNHSignalVector();
            obj.NHSignal=[Sym,Val];
            
            obj.BaseEOM=obj.System.genBaseEOM('Half');
            obj.BaseCons=obj.System.genBaseCons();
            BaseEOM=obj.BaseEOM;
            BaseCons=obj.BaseCons;
            
            cd(obj.Directory);
            save(strcat(obj.System.tagStr('BaseEOM'),'.mat'),'BaseEOM');
            save(strcat(obj.System.tagStr('BaseCons'),'.mat'),'BaseCons');
            cd('..');
            
            obj.IsInitialized=true;
        end
        
        function obj=makeDynamics(obj)
            fileDir=obj.Directory;
            
            flowList=obj.Node.get(obj.Node.Content);
            caseList=num2cell((1:numel(obj.Node.Content)).');
            
            flowProcessList=cell(numel(flowList),1);
            jumpProcessList=cell(numel(flowList),1);
            flowBook=cell(numel(flowList),1);
            jumpBook={};
            
            for ii=1:numel(flowList)
                curFlow=flowList{ii};
                curFlow.makeFlow();
                flowProcessList{ii,1}={strcat('[dq,ds,l]=',curFlow.flowTagStr([]),'(t,q,p,u,s,l);')};
                cd(fileDir);
                flowBook{ii,1}=fReadLine(strcat(curFlow.flowTagStr([]),'.m'));
                nodeJumpList=curFlow.StartEdge.get(curFlow.StartEdge.Content);
                if(isempty(nodeJumpList))
                    jumpProcessList{ii,1}={'%No Jump Here'};
                else
                    curJumpProcess=cell(numel(nodeJumpList),1);
                    for jj=1:numel(nodeJumpList)
                        curJump=nodeJumpList{jj};
                        curJumpProcess{jj,1}={strcat('[act,flow,count,q,p,s]=',curJump.jumpTagStr([]),'(act,flow,count,t,q,p,u,s,l);')};
                        jumpBook=[jumpBook;{fReadLine(strcat(curJump.jumpTagStr([]),'.m'))}];
                    end
                    jumpProcessList{ii,1}=curJumpProcess;
                end
                cd('..');
            end
            
            jumpMapSwitchCase=fGenSwitchList('flow',caseList,jumpProcessList);
            flowMapSwitchCase=fGenSwitchList('flow',caseList,flowProcessList);
            
            cd(fileDir);            
            
            jumpMapName=obj.System.tagStr('Jump');
            flowMapName=obj.System.tagStr('Flow');
            jumpid=fopen(strcat(jumpMapName,'.m'),'w');
            flowid=fopen(strcat(flowMapName,'.m'),'w');
            jumpContent=fReadLine('jumpMapTemplate.tm');
            flowContent=fReadLine('flowMapTemplate.tm');
            
            jumpContent=strrep(jumpContent,'COUNTLIMIT_',num2str(obj.CountLimit));
            jumpContent=strrep(jumpContent,'FUNCNAME_',jumpMapName);
            flowContent=strrep(flowContent,'FUNCNAME_',flowMapName);
            jumpSwitchPos=find(ismember(jumpContent,'%SWITCHCASE_'));
            flowSwitchPos=find(ismember(flowContent,'%SWITCHCASE_'));
            
            fWriteLine(jumpid,jumpContent(1:jumpSwitchPos));
            fWriteLine(flowid,flowContent(1:flowSwitchPos));
            fWriteLine(jumpid,jumpMapSwitchCase,2);
            fWriteLine(flowid,flowMapSwitchCase,1);
            fWriteLine(jumpid,jumpContent(jumpSwitchPos+1:end));
            fWriteLine(flowid,flowContent(flowSwitchPos+1:end));
            
            paramSym=obj.Parameter(:,1);
            paramVal=obj.Parameter(:,2);
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
                sExpr=sym(0);
            else
                s=obj.NHSignal(:,1);
                sExpr=(subs(obj.NHSignal(:,2),paramSym,paramVal));
            end
            l=obj.BaseCons.ConsVector;
            if isempty(l)
                l=sym('CONSSYM');
                allConsJacobian=sym(0);
            else
                allConsJacobian=(subs(obj.BaseCons.ConsJacobian,paramSym,paramVal));
            end
            
            MMatrix=0;
            GFMatrix=0;
            imat=obj.BaseEOM.InertMatrix;
            fmat=obj.BaseEOM.GFMatrix;
            parfor ii=1:numel(imat(1,1,:))
                MMatrix=MMatrix+jSimplify(subs(imat(:,:,ii),paramSym,paramVal));
            end
            parfor ii=1:numel(fmat(1,:))
                GFMatrix=GFMatrix+jSimplify(subs(fmat(:,ii),paramSym,paramVal));
            end
            
            matlabFunction(allConsJacobian,'File','fullConsJacobian.mx','Vars',{t,q,p,u,s,l});
            matlabFunction(GFMatrix,'File','GFMat.mx','Vars',{t,q,p,u,s,l});
            matlabFunction(MMatrix,'File','MMat.mx','Vars',{t,q,p,u,s,l});
            matlabFunction(sExpr,'File','NHSignalODE.mx','Vars',{t,q,p,u,s,l});
            fWriteLine(flowid,[fReadLine('fullConsJacobian.mx');'end'],1);
            fWriteLine(flowid,[fReadLine('GFMat.mx');'end'],1);
            fWriteLine(flowid,[fReadLine('MMat.mx');'end'],1);
            fWriteLine(flowid,[fReadLine('NHSignalODE.mx');'end'],1);
            fWriteLine(jumpid,[fReadLine('MMat.mx');'end'],1);
            delete('*.mx');
            
            for ii=1:numel(jumpBook)
                fWriteLine(jumpid,jumpBook{ii},1);
            end
            
            for ii=1:numel(flowBook)
                fWriteLine(flowid,flowBook{ii},1);
            end
            
            fWriteLine(jumpid,{'end'});
            fWriteLine(flowid,{'end'});
            fclose(jumpid);
            fclose(flowid);
            
            if(obj.IsPrecompile)
                delete(gcp('nocreate'));
                fArg=0;
                tArg=0;
                qArg=zeros(numel(q),1);
                pArg=zeros(numel(p),1);
                uArg=zeros(numel(u),1);
                sArg=zeros(numel(s),1);
                lArg=zeros(numel(l),1);

                codegen(strcat(jumpMapName,'.m'),'-args',{fArg,tArg,qArg,pArg,uArg,sArg,lArg});
                codegen(strcat(flowMapName,'.m'),'-args',{fArg,tArg,qArg,pArg,uArg,sArg,lArg});
            end
            
            cd('..');
            addpath(genpath(pwd));
        end
    end
end