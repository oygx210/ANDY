function [QuatData] = ypr2Quat(yprData)
% Converts Yaw-Pitch-Roll to Quaternion
roll=yprData(1,1);pitch=yprData(2,1); yaw=yprData(3,1);

QuatData=[ 
    cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    ];
end

