% Compiling useful commands into one file

clear all; % Clear all runtime mem
clc; % Clear the command window


% Creating RTDE obj and connecting to the UR socket
% host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
% Constructor of rtde to setup tcp connection
rtde = rtde(host,port);

%{
% Defining points
% Notes these are [x,y,z,r,p,y]
% x,y,z are in mm, as URsim uses mm as well
% r,p,y are in radians
home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];
point1 = [-500, -300, 200, 2.22, -2.22, 1.00];

% Executing the movement.
% movej
[poses1,jointPos1,jointVel1,jointAcc1,jointTor1] = rtde.movej(home);
% movel
[poses2,~] = rtde.movel(point1);
% Not saving movement info
rtde.movel(home);
%}

%{
% Plotting graphs
% You can combine all of the poses like this
poses = [poses1;poses2;];
% and plot the path of the TCP
rtde.drawPath(poses);

% % OR concatenate the poses using the following method
% poses = rtde.movej(home);
% poses = cat(1,poses,rtde.movel(point1));
% poses = cat(1,poses,rtde.movep(home));
% % and plot them as follows
% rtde.drawPath(poses);
%}

%{
% Print out the joint angles at any position.
% Returns in radians
rad2deg(rtde.actualJointPositions())
% Print out the pose at any position. 
% Returns x,y,z in meters
rtde.actualPosePositions()
%}

%{
%
% Drawing a simple square
%
% % Defining points
% % Poses
home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];
point1 = [-588.53, -133.30, 200, 2.2214, -2.2214, 0.00];
point2 = [-688.53, -133.30, 200, 2.2214, -2.2214, 0.00];
point3 = [-688.53, -233.30, 200, 2.2214, -2.2214, 0.00];
point4 = [-588.53, -233.30, 200, 2.2214, -2.2214, 0.00];
% Executing the movement. 
% How does the movement look when you use movel instead?
[poses1,joints1] = rtde.movel(home);
[poses2,joints2]  = rtde.movel(point1);
[poses3,joints3]  = rtde.movel(point2);
[poses4,joints4]  = rtde.movel(point3);
[poses5,joints5]  = rtde.movel(point4);
[poses6,joints6] = rtde.movel(point1);
% Combining both resultant pose lists and plotting graph for square
poses = [poses1;poses2;poses3;poses4;poses5;poses6];
rtde.drawPath(poses)
%}



% Closing the TCP Connection
rtde.close();