classdef kSpace < jNet
    properties(SetAccess=protected)
        System;
        TimeVar;
        RootFrame;
        
        Directory;
        IsPrecompile=true;
    end
    
    methods(Access=public)
        function obj=kSpace(inName,inSys)
            obj@jNet(inName,'kFrame','kLink');
            obj.System=inSys;
            obj.TimeVar=inSys.TimeVar;
            obj.RootFrame=obj.genNode('WORLD').setRoot(true);
            obj.setDir(true); 
            obj.Directory=strcat(obj.System.tagStr('Kinematics'));
            if(~exist(obj.Directory))
                mkdir(obj.Directory);
            end
            cd(obj.Directory);
            obj.Directory=pwd;
            disp(obj.msgStr('Notification',strcat('System Kinematics will be saved in ./',(obj.System.tagStr('Model')))));
            disp(obj.msgStr('Notification','Please put any .stl files related to visualization in the model'));
            cd('..')
        end
        
        function obj=makeKinematics(obj)
            if(~obj.System.Model.IsInitialized)
                obj.System.Model.Init();
            end
            
            cd(obj.Directory);
            frameList=obj.Node.get(obj.Node.Content);
            TFList=sym(zeros(4,4,numel(frameList)));
            TFPath=cell(numel(frameList),2);
            for ii=1:numel(frameList)
                if ~frameList{ii}.IsRoot
                    prevlink=frameList{ii}.EndEdge.get();
                    TFList(:,:,ii)=sym(prevlink{1}.FTF());
                else
                    TFList(:,:,ii)=eye(4,4);
                end
                TFPath(ii,:)={frameList{ii}.NameTag,frameList{ii}.rootChain};
            end
            save(strcat(obj.System.tagStr('Path'),'.mat'),'TFPath');
            
            paramSym=obj.System.Model.Parameter(:,1);
            paramVal=obj.System.Model.Parameter(:,2);
            t=obj.System.TimeVar;
            q=obj.System.Model.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.System.Model.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.System.Model.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.System.Model.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.System.Model.NHSignal(:,1);
            end
            l=obj.System.Model.BaseCons.ConsVector;
            if isempty(l)
                l=sym('CONSSYM');
            end
            
            TFBook=cell(numel(TFList(1,1,:)),1);
            caseList=num2cell((1:numel(obj.Node.Content)).');
            processList=cell(numel(caseList),1);
            for ii=1:numel(TFList(1,1,:))
                TFList(:,:,ii)=subs(TFList(:,:,ii),paramSym,paramVal);
                matlabFunction(TFList(:,:,ii),'File',strcat('TF_',num2str(ii),'.mx'),'Vars',{t,q,p,u,s});
                processList{ii}={strcat('TF=TF_',num2str(ii),'(t,q,p,u,s);')};
                TFBook(ii)={fReadLine(strcat('TF_',num2str(ii),'.mx'))};
            end
            delete('*.mx');
            
            SwitchCase=fGenSwitchList('frame',caseList,processList);
            
            TFMapName=obj.System.tagStr('TF');
            TFid=fopen(strcat(TFMapName,'.m'),'w');
            TFContent=fReadLine('tfMapTemplate.tm');
            TFContent=strrep(TFContent,'FUNCNAME_',TFMapName);
            TFSwitchPos=find(ismember(TFContent,'%SWITCHCASE_'));
            
            fWriteLine(TFid,TFContent(1:TFSwitchPos));
            fWriteLine(TFid,SwitchCase,2);
            fWriteLine(TFid,TFContent(TFSwitchPos+1:end));
            
            for ii=1:numel(TFBook)
                fWriteLine(TFid,[TFBook{ii};'end'],1);
            end
            
            fWriteLine(TFid,{'end'});
            fclose(TFid);            
            
            if(obj.IsPrecompile)
                delete(gcp('nocreate'));
                fArg=0;
                tArg=0;
                qArg=zeros(numel(q),1);
                pArg=zeros(numel(p),1);
                uArg=zeros(numel(u),1);
                sArg=zeros(numel(s),1);

                codegen(strcat(TFMapName,'.m'),'-args',{fArg,tArg,qArg,pArg,uArg,sArg});
            end
            cd('..');
        end
    end
end