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

function T = transform2D(tx, ty, theta_deg)
    theta = deg2rad(theta_deg);
    T = [cos(theta), -sin(theta), tx;
         sin(theta),  cos(theta), ty;
         0,           0,          1];
end


function traj = generateMathWithOperatorColumnTrajectory(expr, hershey, scale, spacing, z_write, z_lift, points_per_segment)
% Generates a trajectory where:
% - Numbers are right-aligned in one column
% - Operator appears in a separate column (next to operand 2)
% Format:
%   10
%    1  -
%    9

    expr = strrep(expr, '=', '');
    result_str = num2str(eval(expr));

    % Parse parts
    tokens = regexp(expr, '(\d+)([\+\-\*])(\d+)', 'tokens');
    if isempty(tokens)
        error('Invalid math expression');
    end

    op1 = tokens{1}{1};       % '10'
    operator = tokens{1}{2};  % '-'
    op2 = tokens{1}{3};       % '1'

    % Determine column width (max digits among all numbers)
    max_digits = max([length(op1), length(op2), length(result_str)]);
    digit_col_x_spacing = 40;
    operator_col_x = max_digits * digit_col_x_spacing + 20;  % Operator in separate column

    % Correct line order: operand1, operand2, result
    number_lines = {
        pad(op1, max_digits, 'left');       % Line 1
        pad(op2, max_digits, 'left');       % Line 2 ← will get operator
        pad(result_str, max_digits, 'left') % Line 3
    };

    op_line_index = 2;  % Place operator with line 2

    traj = [];
    y_offset = 0;

    for i = 1:length(number_lines)
        line = number_lines{i};
        %disp(line(2));

        for j = 1:length(line)
            ch = line(j);
            if ch == ' '
                continue;
            end

            if isempty(hershey{ch})
                warning('Character "%s" not found.', ch);
                continue;
            end

            strokes = densify_strokes(hershey{ch}.stroke, points_per_segment);
            strokes = scale * strokes;
            path = [strokes; zeros(1, size(strokes, 2))];

            k = find(isnan(path(1,:)));
            path(:,k) = path(:,k-1);
            path(3,k) = z_lift / 1000;

            char_traj = path' * 1000;
            char_traj(:,1) = char_traj(:,1) + (j - 1) * digit_col_x_spacing;
            char_traj(:,2) = char_traj(:,2) - y_offset;

            traj = [traj; char_traj];

            % --- Add lift-up and travel segment between characters ---
            if j < length(line)
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
        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if and(i == 2, j == length(line))
                disp(number_lines(j));
                
                % Lift pen at end
                lift_pos = char_traj(end, :); 
                lift_pos(3) = z_lift;
    
                traj = [traj; lift_pos;];
            end
        
        end

        % Add operator in separate column, but only to line 2
        if i == op_line_index
            ch = operator;
            if ~isempty(hershey{ch})
                strokes = densify_strokes(hershey{ch}.stroke, points_per_segment);
                strokes = scale * strokes;
                path = [strokes; zeros(1, size(strokes, 2))];

                k = find(isnan(path(1,:)));
                path(:,k) = path(:,k-1);
                path(3,k) = z_lift / 1000;

                char_traj = path' * 1000;
                char_traj(:,1) = char_traj(:,1) + operator_col_x;
                char_traj(:,2) = char_traj(:,2) - y_offset;

                traj = [traj; char_traj];

                % --- Add lift-up and travel segment between characters ---
                if j < length(line)
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
            end
        end

        y_offset = y_offset + spacing;
        
        if j == length(line)
            % Lift pen at end
            lift_pos = char_traj(end, :); 
            lift_pos(3) = z_lift;

            traj = [traj; lift_pos;];
        end

    end
end


clear all; clc;

% Start RVC module
startup_rvc;
load hershey; % Load hershey fonts

% Create RTDE instance
host = '127.0.0.1';
%host = '192.168.0.100'; % REAL ROBOT
port = 30003;
robo = rtde(host, port);

% Writing params
scale = 0.04;
spacing = 40;
points_per_segment = 2;  % Increase this to get more waypoints

% Movement params
a = 1.4;
v = 0.5;
r = 0.003;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% HEIGHTS
% Considering length of pen
z_write = 30;
z_lift = 1.6 * z_write; % 0.5cm above writing pos
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0.00];
start_pos = [-588.53, -350, z_lift, home(4:6)]; % Start pos of first stroke (mm)

% Move the robot to the home position
robo.movej(home);
%robo.movej(start_pos);

mode = input("Write OR Calculate??(0/1): ");
mode = round(mode);

path = [];


if mode == 0
    % Write only
    disp("-------------------------WRITE MODE--------------------------");
    str = input("Enter 10 alphanumeric characters: ", 's');
    
    if length(str) ~= 10 || ~all(ismember(str, ['0':'9', 'a':'z']))
        error('Invalid input. Must be 10 characters from 0–9 and a–z.');
        return;
    end
    
    myTraj = generateTrajectoryFromString(str, hershey, scale, spacing, z_write, z_lift, points_per_segment);
    
    % Generate a plot of what we are expecting
    scatter3(myTraj(:,1), myTraj(:,2), myTraj(:,3));
    plot3(myTraj(:,1), myTraj(:,2), myTraj(:,3));
    
    
    xOffset = input("Enter the x-offset: ");
    xOffset = round(xOffset);
    yOffset = input("Enter the y-offset: ");
    yOffset = round(yOffset);
    yawOffset = input("Enter the yaw-offset: ");
    yawOffset = round(yawOffset);
    
    % Construct 2D transformation
    T = transform2D(xOffset, yOffset, yawOffset);  % Example values
    
    % Apply to X-Y, leave Z unchanged
    xy = myTraj(:,1:2)';
    xy_h = [xy; ones(1, size(xy,2))];        % Convert to homogeneous (3×N)
    xy_transformed = T * xy_h;              % Transform (3×N)
    
    % Update trajectory
    myTraj(:,1:2) = xy_transformed(1:2,:)';
    
    
    figure;
    plot3(myTraj(:,1), myTraj(:,2), myTraj(:,3), 'b.');
    title('Transformed Trajectory');
    xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on;
    
    %%%%%%%%%%%%%
    for i = 1:size(myTraj,1)
        tcp_xyz = myTraj(i,1:3) + [start_pos(1), start_pos(2), z_write];  % Apply global offset
        
        if tcp_xyz(3) == z_lift + z_write
            tcp_xyz(3) = tcp_xyz(3) - z_write;
        elseif tcp_xyz(3) == 2 * z_write
            tcp_xyz(3) = z_lift;
        end
        
        disp(tcp_xyz(3));
        if tcp_xyz(3) == z_lift
            pose = [tcp_xyz, home(4:6), a, v, 0, 0];
        else
            pose = [tcp_xyz, home(4:6), a, v, 0, r];
        end
        path = [path; pose];
    end

else
    % CALCULATEEE
    disp("---------------MATH STUFF----------------");
    str = input("Enter the math expression:\n", 's');

    traj = generateMathWithOperatorColumnTrajectory(str, hershey, scale, spacing, z_write, z_lift, points_per_segment);
    
    path = [];
    for i = 1:size(traj,1)
        tcp_xyz = traj(i,1:3) + [start_pos(1), start_pos(2), z_write];  % Apply global offset

        if tcp_xyz(3) == z_lift + z_write
            tcp_xyz(3) = tcp_xyz(3) - z_write;
        elseif tcp_xyz(3) == 2 * z_write
            tcp_xyz(3) = z_lift;
        end

        disp(tcp_xyz(3));
        if tcp_xyz(3) == z_lift
            pose = [tcp_xyz, home(4:6), a, v, 0, 0];
        else
            pose = [tcp_xyz, home(4:6), a, v, 0, r];
        end
        path = [path; pose];
    end
    
    %{
    poses = robo.movej(path);
    robo.drawPath(poses);
    %}
end


%
% Execute the movement
poses = robo.movel(path);
robo.movej(home);

robo.drawPath(poses);
%


%{
%
% In simulation only
parfeval(@() robo.movel(path), 0);   % Run motion in background

% Meanwhile, plot path immediately
figure;
plot3(path(:,1), path(:,2), path(:,3), 'b.-');
%
%}


% Close RTDE instance
robo.close();

