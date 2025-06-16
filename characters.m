% Drawing ASCII

function traj = generateTrajectoryFromString(str, hershey, scale, spacing, z_write, z_lift, points_per_segment)
% Generate a 3D trajectory for a given ASCII string using Hershey font
% Returns a [N×3] matrix in mm: [x, y, z]

    traj = [];
    x_offset = 0;

    for c = 1:length(str)
        char_data = hershey{str(c)};
        strokes = densify_strokes(char_data.stroke, points_per_segment);
        strokes = scale * strokes;

        % Add Z layer (initially 0s)
        path = [strokes; zeros(1, size(strokes, 2))];

        % Lift pen at stroke breaks (NaNs)
        k = find(isnan(path(1, :)));
        path(:, k) = path(:, k-1);
        path(3, k) = z_lift / 1000; % In meters

        % Convert to mm and shift in X
        char_traj = path' * 1000;
        char_traj(:,1) = char_traj(:,1) + x_offset;

        traj = [traj; char_traj];
 
        % --- Add lift-up and travel segment between characters ---
        if c < length(str)
            % Lift pen at end
            lift_pos = char_traj(end, :); 
            lift_pos(3) = z_lift;

            % Move in X to next character (with pen lifted)
            move_pos = lift_pos;
            move_pos(1) = move_pos(1) + spacing;

            % Lower pen at next start
            lower_pos = move_pos;
            lower_pos(3) = z_write;

            % Append lift + move + lower to trajectory
            traj = [traj; lift_pos; move_pos; lower_pos];
        end

        % Add spacing between characters
        x_offset = x_offset + spacing;
    end
end


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
spacing = 50;


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

points_per_segment = 10;  % Increase this to get more waypoints

myTraj = generateTrajectoryFromString(str, hershey, scale, spacing, z_write, z_lift, points_per_segment);

% Generate a plot of what we are expecting
scatter3(myTraj(:,1), myTraj(:,2), myTraj(:,3));
plot3(myTraj(:,1), myTraj(:,2), myTraj(:,3));

path = [];
for i = 1:size(myTraj,1)
    tcp_xyz = myTraj(i,1:3) + [start_pos(1), start_pos(2), z_write];  % Apply global offset
    if tcp_xyz(3) > 101
        tcp_xyz(3) = tcp_xyz(3) - 100;
    end
    disp(tcp_xyz(3));
    pose = [tcp_xyz, home(4:6), a, v, 0, r];
    path = [path; pose];
end

% Execute the movement!
poses = robo.movej(path);

robo.drawPath(poses);

% Close RTDE instance
robo.close();
