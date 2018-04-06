function QuatMat=Quat2Mat(inQuat)
% QuatMat=Quat2Mat(inQuat)
% A function that convert Quaternion <inQuat> to Rotation Matrix <QuatMat>.

% Copyright 2018 Robotics & Mechatronics Lab, Virginia Tech.
% Author: Jiamin Wang; Revised: 18-Mar-2018 01:23:47

    qw=inQuat(1);
    qx=inQuat(2);
    qy=inQuat(3);
    qz=inQuat(4);
    QuatMat=...
    [...
    1-2*qy^2-2*qz^2,  2*qx*qy-2*qz*qw,  2*qx*qz+2*qy*qw;...
    2*qx*qy+2*qz*qw,  1-2*qx^2-2*qz^2,  2*qy*qz-2*qx*qw;...
    2*qx*qz-2*qy*qw,  2*qy*qz+2*qx*qw,  1-2*qx^2-2*qy^2;...
    ];
end
