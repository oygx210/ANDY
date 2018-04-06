function [dq,ds,l]=Flow_JoeQPed(flow,t,q,p,u,s,l)
    dq=q*0;
    ds=s*0;
%SWITCHCASE_

	switch flow
	    case 1
	        [dq,ds,l]=Flow_1_JoeQPed(t,q,p,u,s,l);
	end


