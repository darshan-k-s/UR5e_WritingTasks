% Drawing ASCII

function dense_stroke = densify_strokes(stroke, points_per_segment)
    dense_stroke = [];
    current_stroke = [];

    for i = 1:size(stroke, 2) - 1
        p1 = stroke(:, i);
        p2 = stroke(:, i + 1);

        if any(isnan(p1)) || any(isnan(p2))
            % Add current stroke to full output with a NaN separator
            if ~isempty(current_stroke)
                dense_stroke = [dense_stroke, current_stroke, [NaN; NaN]];
                current_stroke = [];
            end
            continue;
        end

        % Linear interpolation between p1 and p2
        interp_x = linspace(p1(1), p2(1), points_per_segment);
        interp_y = linspace(p1(2), p2(2), points_per_segment);
        interp_segment = [interp_x; interp_y];

        current_stroke = [current_stroke, interp_segment];
    end

    % Append last stroke
    if ~isempty(current_stroke)
        dense_stroke = [dense_stroke, current_stroke];
    end
end




clear all; clc;

% Start RVC module
startup_rvc;
load hershey; % Load hershey fonts

% Create RTDE instance
host = '127.0.0.1';
port = 30003;
robo = rtde(host, port);

% Writing params
scale = 0.04;


% Movement params
a = 1.4;
v = 0.5;
r = 0.000;

% Considering length of pen
z_write = 100;
z_lift = 1.05 * z_write; % 0.5cm above writing pos

home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0.00];
start_pos = [-588.53, -350, z_lift, home(4:6)]; % Start pos of first stroke (mm)

% Move the robot to the home position
robo.movej(home);
robo.movej(start_pos);

str = '0123456789';


text = hershey{'2'};
points_per_segment = 10;  % Increase this to get more waypoints
strokes = densify_strokes(text.stroke, points_per_segment);

path = [scale*strokes; zeros(1,numcols(strokes))]; % create the path 

% Where ever there is an nan it indicates that we need to lift up.
k = find(isnan(path(1,:))); %Find index of NaN, i.e, column


path(:,k) = path(:,k-1); path(3,k) = 0.2*scale; % Determine the hight of the lift up motions. 
% 0.2 * scale is the height. 0.2 is in m

traj = [path'*1000]; % convert to the mm units so that we can use the rtde toolbox

% Generate a plot of what we are expecting
scatter3(traj(:,1), traj(:,2), traj(:,3));
plot3(traj(:,1), traj(:,2), traj(:,3));

% Creating a path array
path = [];

for i = 2:size(traj,1)
    disp(traj(1, 1:3));
    tcp_xyz = traj(i,1:3) + [start_pos(1), start_pos(2), z_write];
    %tcp_xyz = traj(i,1:3) + [start_pos(1), start_pos(2), z_write] - traj(1, 1:3);  % or z_lift for pen-up
    point = [tcp_xyz, home(4:6), a, v, 0, r];
    path = [path; point];
end


% Execute the movement!
poses = robo.movej(path);

for i = 1:1000000
end

robo.drawPath(poses);



% Close RTDE instance
robo.close();
