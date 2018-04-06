classdef kSystem < jObj
    properties(SetAccess=protected)
        Space;
        Model;
        
        TimeVar;
        Order=0;
        
        Parameter;  %Constant System Parameters
        Input;      %Input Variables
        Discrete;   %Discrete States
        Continuous; %Continuous States
        NHSignal;   %Nonholonomic Signal
        
        Body;       %Body       (Inertia; Force; Torque)
        Damper;     %Damper
        Potential;  %Potential Energy
        
        Constraint; %Constraint (Will be Converted to Generalized Force with Multiplier)
    end
    
    methods(Access=public)
        function obj=kSystem(inName,inTime)
            obj@jObj(inName);
            
            obj.TimeVar=inTime;
            obj.Model=kModel(obj.tagStr('Model'),obj);
            obj.Space=kSpace(obj.tagStr('Space'),obj);
            addpath(genpath(pwd));
            
            obj.Parameter=jDic(obj.tagStr('Parameter'),'kParam');
            obj.Input=jDic(obj.tagStr('Input'),'kInput');
            obj.Discrete=jDic(obj.tagStr('Discrete'),'kDisc');
            obj.Continuous=jDic(obj.tagStr('Continuous'),'kCont');
            obj.NHSignal=jDic(obj.tagStr('NHSignal'),'kNHSig');
            
            obj.Body=jDic(obj.tagStr('Body'),'kBody');
            obj.Damper=jDic(obj.tagStr('Damper'),'kDamper');
            obj.Potential=jDic(obj.tagStr('Potential'),'kPot');
            
            obj.Constraint=jDic(obj.tagStr('Constraint'),'kCons');
        end
        
        function paramHandle=genParam(obj,inParamSet)
            paramHandle=[];
            if isa(inParamSet,'cell')
                paramSize=size(inParamSet);
                if paramSize(2)==3
                    for ii=1:paramSize(1)
                        curParam=kParam(inParamSet{ii,1},obj,inParamSet{ii,2},inParamSet{ii,3});
                        eval("paramHandle."+char(curParam)+"=curParam;");
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {symbol,description,val;...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {symbol,description,val;...}'));
            end
        end
        
        function inputHandle=genInput(obj,inInputSet)
            inputHandle=[];
            if isa(inInputSet,'cell')
                inputSize=size(inInputSet);
                if inputSize(2)==2
                    for ii=1:inputSize(1)
                        curInput=kInput(inInputSet{ii,1},obj,inInputSet{ii,2});
                        eval("inputHandle."+char(curInput)+"=curInput;");
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {symbol,description,val;...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {symbol,description,val;...}'));
            end
        end
        
        function varargout=genCont(obj,inContSet)
            varargout{1}=[];
            if isa(inContSet,'cell')
                contSize=size(inContSet);
                if contSize(2)==2
                    varargout=cell(contSize(1),1);
                    for ii=1:contSize(1)
                        curCont=kCont(inContSet{ii,1},obj,inContSet{ii,2});
                        varargout{ii}=curCont;
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {(name1) (des1);(name2) (des2);...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {(name1) (des1);(name2) (des2);...}'));
            end
        end
        
        function discHandle=genDisc(obj,inDiscSet)
            discHandle=[];
            if isa(inDiscSet,'cell')
                discSize=size(inDiscSet);
                if discSize(2)==2
                    for ii=1:discSize(1)
                        curDisc=kDisc(inDiscSet{ii,1},obj,inDiscSet{ii,2});
                        eval("discHandle."+char(curDisc)+"=curDisc;");
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {(name1) (des1);(name2) (des2);...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {(name1) (des1);(name2) (des2);...}'));
            end
        end
        
        function varargout=genNHSignal(obj,inNHSignalSet)
            varargout{1}=[];
            if isa(inNHSignalSet,'cell')
                signalSize=size(inNHSignalSet);
                if signalSize(2)==3
                    varargout=cell(signalSize(1),1);
                    for ii=1:signalSize(1)
                        curNHSignal=kNHSig(inNHSignalSet{ii,1},obj,inNHSignalSet{ii,2},inNHSignalSet{ii,3});
                        varargout{ii}=curNHSignal;
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {(name1) (des1) (order1);(name2) (des2) (order2);...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {(name1) (des1) (order1);(name2) (des2) (order2);...}'));
            end
        end
        
        function varargout=genBody(obj,inBodyList)
            if ~(isa(inBodyList,'cell'))
                error(obj.msgStr('Error','Input list of body should be cell array!'));
            end
            
            ListSize=size(inBodyList);
            if ListSize(2)~=2
                error(obj.msgStr('Error','Input list of body should follow format of {name BaseFrame;...}!'));
            end
            
            varargout=cell(ListSize(1),1);
            for ii=1:ListSize(1)
                varargout{ii}=kBody(inBodyList{ii,1},obj,inBodyList{ii,2});
            end
        end
        
        function varargout=genPot(obj,inPotList)
            if ~(isa(inPotList,'cell'))
                error(obj.msgStr('Error','Input list of damper should be cell array!'));
            end
            
            ListSize=size(inPotList);
            if ListSize(2)~=3
                error(obj.msgStr('Error','Input list of damper should follow format of {name dimension power;...}!'));
            end
            
            varargout=cell(ListSize(1),1);
            for ii=1:ListSize(1)
                varargout{ii}=kPot(inPotList{ii,1},obj,inPotList{ii,2},inPotList{ii,3});
            end
        end
        
        function varargout=genDamper(obj,inDamperList)
            if ~(isa(inDamperList,'cell'))
                error(obj.msgStr('Error','Input list of potential enery should be cell array!'));
            end
            
            ListSize=size(inDamperList);
            if ListSize(2)~=3
                error(obj.msgStr('Error','Input list of potential enery should follow format of {name dimension power;...}!'));
            end
            
            varargout=cell(ListSize(1),1);
            for ii=1:ListSize(1)
                varargout{ii}=kDamper(inDamperList{ii,1},obj,inDamperList{ii,2},inDamperList{ii,3});
            end
        end

        function varargout=genCons(obj,inConsList)
            if ~(isa(inConsList,'cell'))
                error(obj.msgStr('Error','Input list of potential enery should be cell array!'));
            end
            
            ListSize=size(inConsList);
            if ListSize(2)~=2
                error(obj.msgStr('Error','Input list of potential enery should follow format of {name symbol;...}!'));
            end
            
            varargout=cell(ListSize(1),1);
            for ii=1:ListSize(1)
                varargout{ii}=kCons(inConsList{ii,1},obj,inConsList{ii,2});
            end
        end
        
        function [Sym,Val]=getParamVector(obj)
            Sym=[];
            Val=[];
            Sym=obj.Parameter.get(obj.Parameter.Content);
            if(isempty(Sym))
                return;
            end
            Val=zeros(numel(Sym),1);
            for ii=1:numel(Sym)
                Val(ii,1)=val(Sym{ii});
            end
            Sym=[Sym{:}].';
        end
        
        function output=getInputVector(obj)
            output=obj.Input.get(obj.Input.Content);
            if(isempty(output))
                return;
            end
            output=[output{:}].';
        end
        
        function output=getContVector(obj,inOrder)
            output=[];
            contList=obj.Continuous.get(obj.Continuous.Content);
            if(isempty(contList))
                return;
            end
            output=sym(zeros(numel(contList),1));
            for ii=1:numel(contList)
                output(ii)=contList{ii}.dot(inOrder);
            end
        end
        
        function output=getDiscVector(obj)
            output=obj.Discrete.get(obj.Discrete.Content);
            if(isempty(output))
                return;
            end
            output=[output{:}].';
        end
        
        function [Sym,Val]=getNHSignalVector(obj)
            Sym=[];
            Val=[];
            signalList=obj.NHSignal.get(obj.NHSignal.Content);
            if(isempty(signalList))
                return;
            end
            
            for ii=1:numel(signalList)
                for jj=0:(signalList{ii}.Order-2)
                    Sym=[Sym;signalList{ii}.dot(jj)];
                    Val=[Val;signalList{ii}.dot(jj+1)];
                end
                Sym=[Sym;signalList{ii}.dot(signalList{ii}.Order-1)];
                Val=[Val;signalList{ii}.Expression];
            end
        end
        
        function obj=setOrder(obj,inOrder)
            if floor(inOrder)>obj.Order
                obj.Order=floor(inOrder);
            end
        end
        
        function Cons=getConsSet(obj,inConsList,varargin)            
            Cons.ConsVector=[];
            Cons.ConsJacobian=[];
            Cons.ConsCoriolis=[];
            Cons.ConsGFMatrix=[];
            
            consList=obj.Constraint.get(inConsList);
            if isempty(consList)
                return;
            else
                Cons.ConsVector=sym(0)*zeros(numel(consList),1);
                Cons.ConsJacobian=sym(0)*zeros(numel(consList),numel(obj.getContVector(1)));
                Cons.ConsCoriolis=sym(0)*zeros(numel(consList),1);
                for ii=1:numel(consList)
                    Cons.ConsVector(ii,1)=consList{ii}.Sym;
                    Cons.ConsJacobian(ii,:)=consList{ii}.Jacobian;
                    Cons.ConsCoriolis(ii,1)=consList{ii}.Coriolis;
                    Cons.ConsGFMatrix=[Cons.ConsGFMatrix,consList{ii}.GFMatrix];
                end
            end
        end
        
        function Cons=genBaseCons(obj)            
            Cons.ConsVector=[];
            Cons.ConsJacobian=[];
            Cons.ConsCoriolis=[];
            Cons.ConsGFMatrix=[];
            
            consList=obj.Constraint.get(obj.Constraint.Content);
            if isempty(consList)
                consList=[];
            else
                consList=[consList{:}];
            end
            Cons.ConsVector=sym(0)*zeros(numel(consList),1);
            jac=sym(0)*zeros(numel(consList),numel(obj.getContVector(1)));
            cor=sym(0)*zeros(numel(consList),1);
            gf=sym(0)*zeros(numel(obj.getContVector(1)),numel(consList));
            parfor ii=1:numel(consList)
                [jac(ii,:),cor(ii,1),gf(:,ii)]=consList(ii).genConsProp();
            end
            for ii=1:numel(consList)
                Cons.ConsVector(ii,1)=consList(ii).Sym;
                consList(ii).setConsProp(jac(ii,:),cor(ii,1),gf(:,ii));
            end
            Cons.ConsJacobian=jac;
            Cons.ConsCoriolis=cor;
            Cons.ConsGFMatrix=gf;
        end
        
        function EOM=genBaseEOM(obj,varargin)
            if nargin==2
                simpStyle=string(varargin{1});
            else
                simpStyle='Full';
            end
            
            dimension=numel(obj.getContVector(1));
            
            bodyList=obj.Body.get();
            damperList=obj.Damper.get();
            potList=obj.Potential.get();
            forceList={};
            torqueList={};
            for ii=1:numel(bodyList)
                forceList=cat(1,forceList,bodyList{ii}.Force.get());
                torqueList=cat(1,torqueList,bodyList{ii}.Torque.get());
            end
            
            bodyList=[bodyList{:}];
            damperList=[damperList{:}];
            potList=[potList{:}];
            forceList=[forceList{:}];
            torqueList=[torqueList{:}];
            forceNumCounter=0;
            
            MMat=sym(zeros(dimension,dimension,numel(bodyList)));
            FMat=sym(zeros(dimension,numel(bodyList)+numel(damperList)+numel(potList)+numel(forceList)+numel(torqueList)));
            curCnt=forceNumCounter;
            parfor ii=1:numel(bodyList)
                forceNumCounter=forceNumCounter+1;
                [MMat(:,:,ii),FMat(:,curCnt+ii)]=bodyList(ii).genInertia(simpStyle);
            end
            for ii=1:numel(bodyList)
                bodyList(ii).setInertia(MMat(:,:,ii),FMat(:,curCnt+ii));
            end
            
            curCnt=forceNumCounter;
            parfor ii=1:numel(damperList)
                forceNumCounter=forceNumCounter+1;
                FMat(:,curCnt+ii)=damperList(ii).genGF(simpStyle);
            end
            for ii=1:numel(damperList)
                damperList(ii).setGF(FMat(:,curCnt+ii));
            end
 
            curCnt=forceNumCounter;
            parfor ii=1:numel(potList)
                forceNumCounter=forceNumCounter+1;
                FMat(:,curCnt+ii)=potList(ii).genGF(simpStyle);
            end
            for ii=1:numel(potList)
                potList(ii).setGF(FMat(:,curCnt+ii));
            end
 
            curCnt=forceNumCounter;
            parfor ii=1:numel(forceList)
                forceNumCounter=forceNumCounter+1;
                FMat(:,curCnt+ii)=forceList(ii).genGF(simpStyle);
            end
            for ii=1:numel(forceList)
                forceList(ii).setGF(FMat(:,curCnt+ii));
            end
            
            curCnt=forceNumCounter;
            parfor ii=1:numel(torqueList)
                forceNumCounter=forceNumCounter+1;
                FMat(:,curCnt+ii)=torqueList(ii).genGF(simpStyle);
            end
            for ii=1:numel(torqueList)
                torqueList(ii).setGF(FMat(:,curCnt+ii));
            end
            
            EOM.InertMatrix=MMat;
            EOM.GFMatrix=FMat;
        end
    end
end