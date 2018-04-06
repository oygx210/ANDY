PathSetup;

%Declare Time Variable and System
t=sym('t','real');
SB=kSystem('SwingBar',t);

%Declare Constant Parameters
C=SB.genParam({
             'g' 'Gravity Acceleration' 9.81;
             'm1' 'Bar 1 Mass' 1;
             'l1' 'Bar 1 Length' 1;
             'm2' 'Bar 2 Mass' 1;
             'l2' 'Bar 2 Length' 1;
             'm3' 'Bar 3 Mass' 0.5;
             'l3' 'Bar 3 Length' 0.5;
             });
           
%Declare System Inputs 
U=SB.genInput({
             'T' 'Bar 1 Torque';
             'b' 'Bar Damper';
             });
            
%Declare Continuous States (Generalized Coordinates) 
[q1,qx,qy,qz]...
=SB.genCont({
            'q1' 'Pan Bar Angle';
            'qx' 'Ball Joint X Rotation';
            'qy' 'Ball Joint Y Rotation';
            'qz' 'Ball Joint Z Rotation';
            });
        
%Declare Coordinate Frames
[Bar1End,Bar1COM,BallJoint,Bar2End,Bar2COM,Bar3End,Bar3COM]...
=SB.Space.genNode({'Bar1End';'Bar1COM';'BallJoint';'Bar2End';'Bar2COM';'Bar3End';'Bar3COM'});

%Declare Kinematic Links
[Root2Bar1End,Bar1End2Bar1COM,Bar1End2BallJoint,BallJoint2Bar2End,Bar2End2Bar2COM,Bar2End2Bar3End,Bar3End2Bar3COM]...
=SB.Space.genEdge({
            'Root2Bar1End' SB.Space.RootFrame.NameTag 'Bar1End';
            'Bar1End2Bar1COM' 'Bar1End' 'Bar1COM';
            'Bar1End2BallJoint' 'Bar1End' 'BallJoint';
            'BallJoint2Bar2End' 'BallJoint' 'Bar2End';
            'Bar2End2Bar2COM' 'Bar2End' 'Bar2COM';
            'Bar2End2Bar3End' 'Bar2End' 'Bar3End';
            'Bar3End2Bar3COM' 'Bar3End' 'Bar3COM';
            });
SB.Space.plotGraph(1);
Root2Bar1End.setDH([0;q1.dot(0);C.l1;0]).simpProp;
Bar1End2Bar1COM.setDH([0;0;-0.5*C.l1;0]).simpProp;
Bar1End2BallJoint.setAngVel([qx.dot(1);qy.dot(1);qz.dot(1)],'bj').simpProp;
BallJoint2Bar2End.setDH([-C.l2;0;0;0]).simpProp;
Bar2End2Bar2COM.setDH([0.5*C.l2;0;0;0]).simpProp;
Bar2End2Bar3End.setDH([0;0;C.l3;0]).simpProp;
Bar3End2Bar3COM.setDH([0;0;-C.l3*0.5;0]).simpProp;
SB.Space.makeKinematics();

%Declare Bodies and Set Inertial Properties
[Bar1,Bar2,Bar3]...
=SB.genBody({
            'Bar1' 'Bar1COM';
            'Bar2' 'Bar2COM';
            'Bar3' 'Bar3COM';
            });
Bar1.setProp(C.m1,diag([0;1/12*C.m1*(C.l1)^2;1/12*C.m1*(C.l1)^2]));
Bar2.setProp(C.m2,diag([1/12*C.m2*(C.l2)^2;1/12*C.m2*(C.l2)^2;0]));
Bar3.setProp(C.m3,diag([0;1/12*C.m3*(C.l3)^2;1/12*C.m3*(C.l3)^2]));
Gravity1=Bar1.genForce({'Gravity1' Bar1COM SB.Space.RootFrame}).setProp([0;0;-C.m1*C.g]);
Gravity2=Bar2.genForce({'Gravity2' Bar2COM SB.Space.RootFrame}).setProp([0;0;-C.m2*C.g]);
Gravity3=Bar3.genForce({'Gravity3' Bar3COM SB.Space.RootFrame}).setProp([0;0;-C.m3*C.g]);
Torque1=Bar1.genTorque({'Torque1' Bar1COM Bar1COM}).setProp([0;0;U.T]);

%Declare Dampers and Set Damper Properties
Bar1Damper=SB.genDamper({'Bar1Damper' 3 1});
Bar1Damper.setProp(U.b*eye(3,3),Bar1.BaseFrame.rootTransVel);
Bar2Damper=SB.genDamper({'Bar2Damper' 3 1});
Bar2Damper.setProp(U.b*eye(3,3),Bar2.BaseFrame.rootTransVel);
Bar3Damper=SB.genDamper({'Bar3Damper' 3 1});
Bar3Damper.setProp(U.b*eye(3,3),Bar3.BaseFrame.rootTransVel);

%Modeling Generation Initialization
[oMode]=SB.Model.Init().genNode({'Open KChain'});
oMode.genFlowEOM({});
SB.Model.plotGraph(2);
SB.Model.makeDynamics();

[contSym]=SB.Model.Continuous;
[sigSym]=SB.Model.NHSignal(:,1);
[paramSym]=SB.Model.Parameter(:,1);
[paramVal]=SB.Model.Parameter(:,2);
Frame=[Bar1End.rootTransDis,Bar2End.rootTransDis,Bar3End.rootTransDis].';
FrameFunc=matlabFunction(subs(Frame(:,1:3),paramSym,paramVal),'Vars',{contSym,sigSym});
freq=1000;
h=1/freq;
tspan=0:h:30;
input=[1;0.2];
odeState=[0;0;0;0;-5;0;0;0];
stateRecord=zeros(8,numel(tspan));
sigState=eul2quat([0 pi/2 0]).';
sigRecord=zeros(4,numel(tspan));
posRecordX=zeros(3,numel(tspan));
posRecordY=zeros(3,numel(tspan));
posRecordZ=zeros(3,numel(tspan));
curPos=FrameFunc(odeState,sigState);
figure(3)
handle=plot3([0;curPos(:,1)],[0;curPos(:,2)],[0;curPos(:,3)],'k','LineWidth',10);
axis([-3,3,-3,3,-2,2]);
stateRecord(:,1)=odeState;
sigRecord(:,1)=sigState;
posRecordX(:,1)=curPos(:,1);
posRecordY(:,2)=curPos(:,2);
posRecordZ(:,3)=curPos(:,3);
tic
for ii=2:numel(tspan)
    [odeState,sigState,l1]=rk4Hybrid(@Flow_SwingBar_mex,h,1,tspan(ii),odeState,0,input,sigState,0);
    curPos=FrameFunc(odeState,sigState);
    if(rem(ii,30)==0)
        handle.set('XData',[0;curPos(:,1)],'YData',[0;curPos(:,2)],'ZData',[0;curPos(:,3)]);
        drawnow
    end
    stateRecord(:,ii)=odeState;
    sigRecord(:,ii)=sigState;
    posRecordX(:,ii)=curPos(:,1);
    posRecordY(:,ii)=curPos(:,2);
    posRecordZ(:,ii)=curPos(:,3);
end
toc


fig=figure(5);
sub1=subplot(4,1,1);
hold on
axis([-3,3,-3,3,-2,2],'equal');
sub1.View=[45,5];
plot3([0;curPos(:,1)],[0;curPos(:,2)],[0;curPos(:,3)],'g','LineWidth',5);
plot3(posRecordX(1,1:100:end),posRecordY(1,1:100:end),posRecordZ(1,1:100:end),'-.k','LineWidth',2)
plot3(posRecordX(2,1:100:end),posRecordY(2,1:100:end),posRecordZ(2,1:100:end),'r','LineWidth',1)
plot3(posRecordX(3,1:100:end),posRecordY(3,1:100:end),posRecordZ(3,1:100:end),':b','LineWidth',1)
title('3D Trajectory in World Frame')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('Bodies','H.Bar End Traj','BS.Bar End Traj','ES.Bar End Traj','location','southoutside')
axis('image')
hold off
sub2=subplot(4,1,2);
hold on
plot(tspan(1:500:end),sigRecord(1,1:500:end),'-or','LineWidth',1);
plot(tspan(1:500:end),sigRecord(2,1:500:end),'-*g','LineWidth',1);
plot(tspan(1:500:end),sigRecord(3,1:500:end),'-b','LineWidth',1);
plot(tspan(1:500:end),sigRecord(4,1:500:end),'--k','LineWidth',1);
title('Quaternion of Ball Joint Rotation')
xlabel('Time (s)')
ylabel('Value')
legend('Qw','Qx','Qy','Qz','location','northoutside','Orientation','horizontal')
hold off
sub3=subplot(4,1,3);
hold on
plot(tspan(1:500:end),posRecordX(1,1:500:end),'-og','LineWidth',1);
plot(tspan(1:500:end),posRecordX(2,1:500:end),'-*b','LineWidth',1);
plot(tspan(1:500:end),posRecordX(3,1:500:end),':r','LineWidth',3);
title('X Coordinate Trajectory of Bar End Points')
xlabel('Time (s)')
ylabel('X (m)')
legend('H.Bar','BS.Bar','ES.Bar','location','northoutside','Orientation','horizontal')
hold off
sub4=subplot(4,1,4);
hold on
plot(tspan(1:500:end),posRecordZ(1,1:500:end),'-*g','LineWidth',1);
plot(tspan(1:500:end),posRecordZ(2,1:500:end),'-ob','LineWidth',1);
plot(tspan(1:500:end),posRecordZ(3,1:500:end),':r','LineWidth',3);
title('Z Coordinate Trajectory of Bar End Points')
xlabel('Time (s)')
ylabel('Z (m)')
legend('H.Bar','BS.Bar','ES.Bar','location','northoutside','Orientation','horizontal')
hold off

set(sub1,'FontName','Times New Roman','FontWeight','bold')
set(sub2,'FontName','Times New Roman','FontWeight','bold')
set(sub3,'FontName','Times New Roman','FontWeight','bold')
set(sub4,'FontName','Times New Roman','FontWeight','bold')