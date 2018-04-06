classdef jNet < jObj
    % jNet is the wrapper of graph network to provide utility required in ANDY.
    
    properties(SetAccess=protected)
        Node;
        Graph;
        EditFunc;
        Axes;
        Handle;
        
        EdgeLabel=true;
        GraphDir=false;
        NodeType='jNode';
        EdgeType='jEdge';
    end
    
    properties(Access=protected)
        UpdateFlag=true;
    end
    
    methods(Access=public)
        function obj=jNet(inName,varargin)
            obj@jObj(inName);
            
            if nargin==3
                obj.NodeType=varargin{1};
                obj.EdgeType=varargin{2};
            elseif (nargin~=1)&&(nargin~=3)
                error(obj.msgStr('Error','Constructor for jNet requires (Name) or (Name,NodeClass,EdgeClass)!'));
            end
            
            obj.Node=jDic(obj.tagStr('Node'),obj.NodeType);
            obj.genGraph();
            obj.EditFunc=@(obj,nodeTable,edgeTable) defaultEditFunc(obj,nodeTable,edgeTable);
        end
        
        function obj=setDir(obj,inOpt)
            if inOpt
                obj.GraphDir=true;
            else
                obj.GraphDir=false;
            end
        end
        
        function obj=setEdgeLabel(obj,inOpt)
            if inOpt
                obj.EdgeLabel=true;
            else
                obj.EdgeLabel=false;
            end
        end
        
        function varargout=genNode(obj,inNode)
            if isa(inNode,'char')
                eval("newNode="+obj.NodeType+"('"+inNode+"',obj);");
                varargout{1}=newNode;
            elseif isa(inNode,'cell')
                nodeNum=numel(inNode);
                varargout=cell(nodeNum,1);
                for ii=1:nodeNum
                    varargout{ii}=obj.genNode(inNode{ii});
                end
            else
                error(obj.msgStr('Error','Node''s NameTag should be char of a cell of char'));
            end
            obj.UpdateFlag=true;
        end
        
        function varargout=genEdge(obj,edgeTable)
            if isa(edgeTable,'cell')
                edgeName=edgeTable(:,1);
                startNode=edgeTable(:,2);
                endNode=edgeTable(:,3);
                edgeNum=numel(edgeName);
                varargout=cell(edgeNum,1);
                for ii=1:edgeNum
                    if (isa(edgeName{ii},'char')&&isa(startNode{ii},'char')&&isa(endNode{ii},'char'))
                        eval("newEdge="+obj.EdgeType+"('"+edgeName{ii}+"',obj.Node.get('"...
                            +startNode{ii}+"'),obj.Node.get('"+endNode{ii}+"'));");
                        varargout{ii}=newEdge;
                    else
                        error(obj.msgStr('Error','Input follows the form of {Name1 StartFrame1 EndFrame1;...}'));
                    end
                end
            else
                error(obj.msgStr('Error','Input follows the form of {Name1 StartFrame1 EndFrame1;...}'));
            end

            obj.UpdateFlag=true;
        end
        
        function output=collectEdge(obj)
            nodeList=obj.Node.get(obj.Node.Content);
            nodeNum=numel(nodeList);
            output=[];
            for ii=1:nodeNum
                output=[output;nodeList{ii}.StartEdge.get()];
            end
        end
        
        function output=path(obj,node1,node2)
            if(isa(node1,'jNode'))
                node1=node1.NameTag;
            end
            if(isa(node2,'jNode'))
                node2=node2.NameTag;
            end
            obj.genGraph();
            nodePath=shortestpath(obj.Graph,node1,node2);
            nodeNum=numel(nodePath);
            output=[];
            for ii=1:(nodeNum-1)
                output=[output;obj.Node.get(nodePath{ii}).StartEdge.get({nodePath{ii+1}})];
            end
        end
        
        function [nodeTable,edgeTable]=genGraph(obj)
            nodeTable=[];
            edgeTable=[];
            
            if nargout==2
                if(obj.UpdateFlag)
                    if obj.GraphDir
                        obj.Graph=digraph();
                    else
                        obj.Graph=graph();
                    end
                end
                
                if(numel(obj.Node.Content)>0)
                    nodeTable=obj.Node.Content;
                    if(obj.UpdateFlag)
                        obj.Graph=addnode(obj.Graph,nodeTable);
                    end
                end

                edgeList=obj.collectEdge();
                edgeNum=numel(edgeList);
                if(edgeNum>0)
                    edgeTable=cell(edgeNum,3);
                    for ii=1:edgeNum
                        edgeTable{ii,1}=edgeList{ii}.NameTag;
                        edgeTable{ii,2}=edgeList{ii}.StartNode.NameTag;
                        edgeTable{ii,3}=edgeList{ii}.EndNode.NameTag;
                        edgeTable{ii,4}=edgeList{ii}.StartNode.NodeNumber;
                        edgeTable{ii,5}=edgeList{ii}.EndNode.NodeNumber;
                    end
                    if(obj.UpdateFlag)
                        obj.Graph=addedge(obj.Graph,edgeTable(:,2),edgeTable(:,3));
                    end
                end
            end
        end
        
        function obj=plotGraph(obj,figureNum)
            delete(obj.Axes);
            figure(figureNum);
            obj.Axes=axes();
            [nodeTable,edgeTable]=obj.genGraph();
            obj.Handle=plot(obj.Axes,obj.Graph);
            obj.Handle=obj.EditFunc(obj,nodeTable,edgeTable);
        end
        
        function GPHandle=defaultEditFunc(obj,nodeTable,edgeTable)
            GPHandle=obj.Handle;
            set(obj.Axes,'Visible','off');
            axis('image');
            set(GPHandle,'EdgeAlpha',0.3);
            set(GPHandle,'LineWidth',2);
            set(GPHandle,'MarkerSize',20);
            set(GPHandle,'NodeColor',[0.8 0.8 0.8]);
            set(GPHandle,'EdgeColor',[0 0 0]);
            set(GPHandle,'ArrowSize',10);
            text(obj.Axes,GPHandle.XData,GPHandle.YData,GPHandle.NodeLabel,'HorizontalAlignment','center','FontName','Courier New','FontWeight','bold');
            if(~isempty(edgeTable))&&obj.EdgeLabel
                for ii=1:numel(edgeTable(:,1))
                    centerPos=[(GPHandle.XData(edgeTable{ii,4})+GPHandle.XData(edgeTable{ii,5}))/2,(GPHandle.YData(edgeTable{ii,4})+GPHandle.YData(edgeTable{ii,5}))/2];
                    ang=atan2((GPHandle.YData(edgeTable{ii,5})-GPHandle.YData(edgeTable{ii,4})),(GPHandle.XData(edgeTable{ii,5})-GPHandle.XData(edgeTable{ii,4})));
                    if ang>pi/2
                        ang=ang-pi;
                    elseif ang<-pi/2
                        ang=ang+pi;
                    end
                    centerPos=centerPos+0.05*[cos(ang+pi/2),sin(ang+pi/2)];
                    txtline=edgeTable{ii,1};
                    writeline=[];
                    lineMax=10;
                    while(1)
                        if numel(txtline)>lineMax*1.5
                            if(txtline(lineMax+1)~=char(32))&&(txtline(lineMax)~=char(32))
                                writeline=[writeline txtline(1:lineMax) '-' char(10)];
                            else
                                writeline=[writeline txtline(1:lineMax) char(10)];
                            end
                            txtline=txtline(lineMax+1:end);
                        else
                            writeline=[writeline txtline];
                            break;
                        end
                    end
                    text(obj.Axes,centerPos(1),centerPos(2),writeline,'HorizontalAlignment','center','VerticalAlignment','bottom','Rotation',ang*180/pi,'FontName','Courier New','FontWeight','bold');
                end
            end
            set(GPHandle,'NodeLabel',[])
            drawnow;
        end
    end
end