PathSetup;

Sim=sAxes('Joe''s Quadruped Simulator',3,'Path_JoeQPed.mat',@TF_JoeQPed_mex);
Sim.setAxesProp('TorsoFrame',[-1 -1.5 -0.75; 1 1 0.25],[210 15]).setPresetTF('WORLD');

[LFLegPose,RFLegPose,LRLegPose,RRLegPose,TorsoPose,TailPose]...
    =Sim.genPlot({'LFLegPose';'RFLegPose';'LRLegPose';'RRLegPose';'TorsoPose';'TailPose'});
LFLegPose.setLineSpec('-','r',3).setPlotPoint({'LegLFThighJoint';'LegLFShankJoint';'LegLFFoot'});
RFLegPose.setLineSpec('-','r',3).setPlotPoint({'LegRFThighJoint';'LegRFShankJoint';'LegRFFoot'});
LRLegPose.setLineSpec('-','r',3).setPlotPoint({'LegLRThighJoint';'LegLRShankJoint';'LegLRFoot'});
RRLegPose.setLineSpec('-','r',3).setPlotPoint({'LegRRThighJoint';'LegRRShankJoint';'LegRRFoot'});
TorsoPose.setLineSpec('-','b',3).setPlotPoint({'LegLFHipJoint';'LegRFHipJoint';'LegRRHipJoint';'LegLRHipJoint';'LegLFHipJoint'});
TailPose.setLineSpec('-','g',3).setPlotPoint({'TailYaw';'TailEnd'});

initTorsoPos=[0;0;0.5;0;0;0];
initLegPos=[0.6;-0.5857;2*0.5857];
initTailPos=[pi/6;pi/6];
initPos=[initTorsoPos;initLegPos;initLegPos;initLegPos;initLegPos;initTailPos];
initVel=zeros(20,1);
Sim.drawNow(0,[initPos;initVel],zeros(8,1),zeros(14,1),[1;0;0;0],0);