PathSetup;
% close all;
Sim=sAxes('Joe''s Quadruped Simulator',3,'Path_JoeQPed.mat',@numTF_JoeQPed_mex);
Sim.setAxesProp('TorsoFrame',[-0.75 -1.25 -0.75; 0.75 0.75 0.75],[210 15]).setPresetTF('WORLD');

[LFLegPose,RFLegPose,LRLegPose,RRLegPose,TorsoPose,TailPose]...
    =Sim.genPlot({'LFLegPose';'RFLegPose';'LRLegPose';'RRLegPose';'TorsoPose';'TailPose'});
LFLegPose.setLineSpec('-','none','r',3).setPlotPoint({'LegLFThighJoint';'LegLFShankJoint';'LegLFFoot'});
RFLegPose.setLineSpec('-','none','r',3).setPlotPoint({'LegRFThighJoint';'LegRFShankJoint';'LegRFFoot'});
LRLegPose.setLineSpec('-','none','r',3).setPlotPoint({'LegLRThighJoint';'LegLRShankJoint';'LegLRFoot'});
RRLegPose.setLineSpec('-','none','r',3).setPlotPoint({'LegRRThighJoint';'LegRRShankJoint';'LegRRFoot'});
TorsoPose.setLineSpec('-','none','b',3).setPlotPoint({'LegLFHipJoint';'LegRFHipJoint';'LegRRHipJoint';'LegLRHipJoint';'LegLFHipJoint'});
TailPose.setLineSpec('-','none','g',3).setPlotPoint({'TailYaw';'TailCOM'});

[Floor,Torso,Tail,LFThigh,LFShank,RFThigh,RFShank,LRThigh,LRShank,RRThigh,RRShank]...
=Sim.genPatch({'Floor'  'WORLD';...
               'Torso' 'TorsoFrame';...
               'Tail' 'TailCOM';...
               'LFThigh' 'LegLFThighCOM';...
               'LFShank' 'LegLFShankCOM';...
               'RFThigh' 'LegRFThighCOM';...
               'RFShank' 'LegRFShankCOM';...
               'LRThigh' 'LegLRThighCOM';...
               'LRShank' 'LegLRShankCOM';...
               'RRThigh' 'LegRRThighCOM';...
               'RRShank' 'LegRRShankCOM';...
               });
Floor.setFaceProp([0.8 0.8 1],0.75).setModel('Floor.STL',1,[],0.001);
Torso.setFaceProp('b',0.75).setModel('Torso.STL',1,[],0.001);
Tail.setFaceProp('g',0.75).setModel('Tail.STL',1,[],0.001);
LFThigh.setFaceProp('r',0.75).setModel('Thigh.STL',1,[],0.001);
LFShank.setFaceProp('y',0.75).setModel('Shank.STL',1,[],0.001);
RFThigh.setFaceProp('r',0.75).setModel('Thigh.STL',1,[],0.001);
RFShank.setFaceProp('y',0.75).setModel('Shank.STL',1,[],0.001);
LRThigh.setFaceProp('r',0.75).setModel('Thigh.STL',1,[],0.001);
LRShank.setFaceProp('y',0.75).setModel('Shank.STL',1,[],0.001);
RRThigh.setFaceProp('r',0.75).setModel('Thigh.STL',1,[],0.001);
RRShank.setFaceProp('y',0.75).setModel('Shank.STL',1,[],0.001);




tic
freq=1000;
h=1/freq;
flow=2;
tspan=0:h:0.25;
initTorsoPos=[0.2;0.3;0.4;0;0;0];
initLegPos2=[0.0749;-0.8386-0.1391;1.6570];
initLegPos1=[0.0749;-0.8386;1.6771];
initLegPos3=[0.0749;-0.8386;1.6771];
initLegPos4=[0.0749;-0.8386+0.1391;1.6570];
initTailPos=[0;0];
initPos=[initTorsoPos;initLegPos1;initLegPos2;initLegPos3;initLegPos4;initTailPos];
initVel=zeros(20,1);
contState=[initPos;initVel];
discState=[-0.01;0.57;0.41;0.63;-0.01;0.03;0.41;-0.03;0;100];
consForce=zeros(12,1);
legForce=[0;0;-15];
input=zeros(14,1);
input=[legForce;legForce;legForce/15*20.5;legForce/15*20.5;-7.7;-10*sin(31.4*0)];
nhSignal=[1;0;0;0];
Sim.drawNow(0,[initPos;initVel],zeros(10,1),zeros(14,1),nhSignal);
Sim.FigHandle.set('outerposition',[0 0 1000 1000])
f = getframe;
[im,map] = rgb2ind(f.cdata,256,'nodither');
im(:,:,1,1) = 0;
k=0;

XYZ=diag([0.2;0.3;0.4])*ones(3,numel(tspan));
for ii=2:numel(tspan)
    [contState,nhSignal,consForce]=rk4Hybrid(@Flow_JoeQPed_mex,h,flow,tspan(ii),contState,discState,input,nhSignal,consForce);
    
    input=[legForce;legForce;legForce/15*20.5;legForce/15*20.5;-7.7;-10*sin(31.4*ii*h/2)];

    if(rem(ii,10)==0)
        Sim.drawNow(tspan(ii),contState,discState,input,nhSignal);
        f = getframe;
        k=k+1;
        im(1:size(f.cdata)*[1;0;0],1:size(f.cdata)*[0;1;0],1,k) = rgb2ind(f.cdata,map,'nodither');
    end
end
toc
im=im(180:end-120,120:end-120,:,:);
imwrite(im,map,'QPed.gif','DelayTime',0,'LoopCount',inf) %g443800
% 
% figure(20)
% hold on
% plot(tspan,XYZ(1,:));
% plot(tspan,XYZ(2,:));
% plot(tspan,XYZ(3,:));
