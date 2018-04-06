function [de_EqSum,de_Eq]=pDiff(f,xVar)
% [de_EqSum,de_Eq]=pDiff(f,xVar)
% This function is used to calculate the partial derivative of a set of
% expressions <f> with respect to a dependent variable <xVar> To use this function
% requires the correct naming protocol:
%   {x}__d{@}_{$} 
% Here {x} is the actual variable name {@} is the derivative variable
% xVar, {$} is the rank of derivative.
% [de_EqSum] is the default output, [de_Eq] is a secondary output that gives
% derivative of each <xVar> dependent variable.

% Copyright 2018 Robotics & Mechatronics Lab, Virginia Tech.
% Author: Jiamin Wang; Revised: 18-Mar-2018 01:23:48

    % Get all the variables for differentiation
    x0=symvar(f);
    x0(end+1)=xVar;
    x0=unique(x0);
        
    % Differentiation
    if(max(size(xVar))>1)
        error('Currently Only Support Partial Differentiation According to One Variable')
    else
        diffSign=strcat('__d',char(xVar),'_');
        fSize=size(f);
        x0Size=numel(x0);
        de_EqSum=sym(zeros(fSize(1),fSize(2)));
        de_Eq=sym(zeros(fSize(1),fSize(2),x0Size));
        for ii=1:x0Size
            diff_Eq=diff(f,x0(ii));    
            x_Str=char(x0(ii));
            str_len=numel(x_Str);
            rank_Pos=strfind(x_Str,diffSign);
            rank_Length=length(diffSign);
            
            if(isempty(rank_Pos))
                if(x0(ii)==xVar)
                    de_Eq(:,:,ii)=diff_Eq;%If the current variable is xVar
                else
                    de_Eq(:,:,ii)=zeros(fSize);%If the current variable is xVar-independent
                end
            else%If the current variable is xVar-dependent
                rank_Pos=rank_Pos(end)+rank_Length;
                rank_num=floor(str2double(x_Str(rank_Pos:str_len))+1);
                dx_Str=strcat(x_Str(1:rank_Pos-1),num2str(rank_num));
                de_Eq(:,:,ii)=diff_Eq*sym(dx_Str);
            end
            
            de_EqSum=de_EqSum+de_Eq(:,:,ii);
        end
    end
end
