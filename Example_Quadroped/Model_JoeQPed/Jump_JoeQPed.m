function [flow,flowTraj,q,p,s]=Jump_JoeQPed(flow,t,q,p,u,s,l)
    act=0;
    count=0;
    flowTraj=zeros(100+1,1);
    while(1)
        flowTraj(count+1)=flow;
        act=0;
%SWITCHCASE_

		switch flow
		    case 1
		        %No Jump Here
		end

        if(act==0)
            break;
        end
        if(count>100)
            flow=-flow;%Enter zeno or other infinite jump problem, quit and give an error.
            break;
        end
    end

