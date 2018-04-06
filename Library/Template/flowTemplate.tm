function [dq,ds,l]=FUNCNAME_(t,q,p,u,s,l)
    l=lambdaUpdate(t,q,p,u,s,l);    
    qSize=numel(q);
    MM=MMat(t,q,p,u,s,l);
    dq=[q((qSize/2+1):qSize);MM\(GFMat(t,q,p,u,s,l)+consGFMat(t,q,p,u,s,l)+fullConsJacobian(t,q,p,u,s,l).'*l)];
    ds=NHSignalODE(t,q,p,u,s,l);

    function l=lambdaUpdate(t,q,p,u,s,l)
        l=l*0;
        lActive=activeConsNum();
        M=MMat(t,q,p,u,s,l);
        J=partialConsJacobian(t,q,p,u,s,l);
        if ~(lActive==0)
            l(lActive)=-(J*(M\J.'))\(J*(M\GFMat(t,q,p,u,s,l))+consCorMat(t,q,p,u,s,l));
        end
    end