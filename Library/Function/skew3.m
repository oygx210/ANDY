function output=skew3(w)
% output=skew3(w)
% Convert 3x1 vector <w> to a skew matrix <output>.

% Copyright 2018 Robotics & Mechatronics Lab, Virginia Tech.
% Author: Jiamin Wang; Revised: 18-Mar-2018 01:23:48

    if(numel(w)==3)
        output=[0 -w(3) w(2) ; w(3) 0 -w(1) ; -w(2) w(1) 0 ];
    else
        error('Cannot convert to Skew Symmetric Matrix unless is a 3x1 vector');
    end
end
