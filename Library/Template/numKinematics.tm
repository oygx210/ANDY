function [frameTF,frameVel,frameCorAcc,frameJacobian,frameTransDis]=FUNCNAME_(t,q,p,u,s)
    LinkTF=linkTF(t,q,p,u,s);
    LinkVel=linkVel(t,q,p,u,s);
    LinkCorAcc=linkCorAcc(t,q,p,u,s);
    LinkJacobian=linkJacobian(t,q,p,u,s);

    FrameNum=FRAMENUM_;
    DOF=numel(q)/2;
    
    LinkTF=reshape((LinkTF),[4,4,FrameNum]);
    LinkJacobian=reshape((LinkJacobian),[6,DOF,FrameNum]);
    
    frameTF=zeros(4,4,FrameNum);
    frameTransDis=zeros(3,FrameNum);
    frameVel=zeros(6,FrameNum);
    frameCorAcc=zeros(6,FrameNum);
    frameJacobian=zeros(6,DOF,FrameNum);
    
    frameTF(:,:,1)=LinkTF(:,:,1);
    frameTransDis(:,1)=frameTF(1:3,4,1);
    frameVel(:,1)=LinkVel(:,1);
    frameCorAcc(:,1)=LinkCorAcc(:,1);
    frameJacobian(:,:,1)=LinkJacobian(:,:,1);

    frameCheckList=zeros(1,FrameNum);
    frameCheckList(1)=1;

    for frame=1:FrameNum
        framePath=0;
        %{
        thisTF=eye(4,4);
        thisTransVel=zeros(3,1);
        thisCorTransAcc=zeros(3,1);
        thisTransJacobian=zeros(3,DOF);
        thisAngVel=zeros(3,1);
        thisCorAngAcc=zeros(3,1);
        thisAngJacobian=zeros(3,DOF);
        %}

%SWITCHCASE_

        [frameCheckList,LinkTF,frameTF,LinkVel,frameVel,LinkCorAcc,frameCorAcc,LinkJacobian,frameJacobian,frameTransDis]=frameKineRecur(framePath,frameCheckList,LinkTF,frameTF,LinkVel,frameVel,LinkCorAcc,frameCorAcc,LinkJacobian,frameJacobian,frameTransDis);

        %{
        for jj=numel(framePath):-1:1
            curNum=framePath(jj);
            curTF=LinkTF(:,:,curNum);
            curTransVel=LinkVel(1:3,curNum);
            curAngVel=LinkVel(4:6,curNum);
            curCorTransAcc=LinkCorAcc(1:3,curNum);
            curCorAngAcc=LinkCorAcc(4:6,curNum);
            curTransJacobian=LinkJacobian(1:3,:,curNum);
            curAngJacobian=LinkJacobian(4:6,:,curNum);
            
            curRotMat=curTF(1:3,1:3);
            thisTransDis=curRotMat*thisTF(1:3,4);

            curRadVel=cross(curAngVel,thisTransDis);
            curRotAngVel=curRotMat*thisAngVel;
            thisRotTransVel=curRotMat*thisTransVel;
            
            thisCorTransAcc=curCorTransAcc...
                            +cross(curCorAngAcc,thisTransDis)...
                            +curRotMat*thisCorTransAcc...
                            +2*cross(curAngVel,thisRotTransVel)...
                            +cross(curAngVel,curRadVel);
            thisCorAngAcc=  curCorAngAcc...
                            +curRotMat*thisCorAngAcc...
                            +cross(curAngVel,curRotAngVel);
                        
            thisTransVel=   curTransVel...
                            +thisRotTransVel...
                            +curRadVel;
            thisAngVel=curAngVel+curRotAngVel;
            

            skewTransDis=[0 -thisTransDis(3) thisTransDis(2) ; thisTransDis(3) 0 -thisTransDis(1) ; -thisTransDis(2) thisTransDis(1) 0 ];

            thisTransJacobian=  curTransJacobian...
                                +curRotMat*thisTransJacobian...
                                -skew3(thisTransDis)*curAngJacobian;
            thisAngJacobian=curAngJacobian+curRotMat*thisAngJacobian;
            
            thisTF=curTF*thisTF;
        end
        
        frameTF(:,:,frame)=thisTF;
        frameTransDis(:,frame)=thisTF(1:3,4);
        frameVel(:,frame)=[thisTransVel;thisAngVel];
        frameCorAcc(:,frame)=[thisCorTransAcc;thisCorAngAcc];
        frameJacobian(:,:,frame)=[thisTransJacobian;thisAngJacobian];
        %}
    end

    function [frameCheckList_,TF_,frameTF_,Vel_,frameVel_,CorAcc_,frameCorAcc_,Jacobian_,frameJacobian_,frameTransDis_]=frameKineRecur(framePath_,frameCheckList_,TF_,frameTF_,Vel_,frameVel_,CorAcc_,frameCorAcc_,Jacobian_,frameJacobian_,frameTransDis_)
        curNum_=framePath_(end);
        if(~frameCheckList_(curNum_))
            preNum_=framePath_(end-1);
            if(frameCheckList_(preNum_))
                frameTF_(:,:,curNum_)=frameTF_(:,:,preNum_)*TF_(:,:,curNum_);
                frameTransDis_(:,curNum_)=frameTF_(1:3,4,curNum_);

                curTransVel=Vel_(1:3,curNum_);
                curAngVel=Vel_(4:6,curNum_);
                curCorTransAcc=CorAcc_(1:3,curNum_);
                curCorAngAcc=CorAcc_(4:6,curNum_);
                curTransJacobian=Jacobian_(1:3,:,curNum_);
                curAngJacobian=Jacobian_(4:6,:,curNum_);

                preTransVel=frameVel_(1:3,preNum_);
                preAngVel=frameVel_(4:6,preNum_);
                preCorTransAcc=frameCorAcc_(1:3,preNum_);
                preCorAngAcc=frameCorAcc_(4:6,preNum_);
                preTransJacobian=frameJacobian_(1:3,:,preNum_);
                preAngJacobian=frameJacobian_(4:6,:,preNum_);
                
                preRotMat=frameTF_(1:3,1:3,preNum_);
                preRotCurTransDis=preRotMat*TF_(1:3,4,curNum_);

                frameVel_(1:3,curNum_)=preTransVel+preRotMat*curTransVel+cross(preAngVel,preRotCurTransDis);
                frameVel_(4:6,curNum_)=preAngVel+preRotMat*curAngVel;

                frameCorAcc_(1:3,curNum_)=preCorTransAcc+cross(preAngVel,cross(preAngVel,preRotCurTransDis))...
                                        +preRotMat*curCorTransAcc+cross(preCorAngAcc,preRotCurTransDis)...
                                        +2*cross(preAngVel,preRotMat*curTransVel);
                frameCorAcc_(4:6,curNum_)=preCorAngAcc+preRotMat*curCorAngAcc+cross(preAngVel,preRotMat*curAngVel);

                skewTransDis=[0 -preRotCurTransDis(3) preRotCurTransDis(2) ; preRotCurTransDis(3) 0 -preRotCurTransDis(1) ; -preRotCurTransDis(2) preRotCurTransDis(1) 0 ];

                frameJacobian_(1:3,:,curNum_)=preTransJacobian+preRotMat*curTransJacobian-skewTransDis*preAngJacobian;
                frameJacobian_(4:6,:,curNum_)=preAngJacobian+preRotMat*curAngJacobian;

                frameCheckList_(curNum_)=1;
            else
                [frameCheckList_,TF_,frameTF_,Vel_,frameVel_,CorAcc_,frameCorAcc_,Jacobian_,frameJacobian_,frameTransDis_]=frameKineRecur(framePath_(1:end-1),frameCheckList_,TF_,frameTF_,Vel_,frameVel_,CorAcc_,frameCorAcc_,Jacobian_,frameJacobian_,frameTransDis_);
            end
        end
    end