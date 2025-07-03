% UR5e ASCII writing
% Author: Darshan K S
% For MTRN4230 2025

function traj = makeTrajFromStr(str, hershey, scale, spacing, zWrite, zLift, pointsperStroke)
% Create a trajectory for given ASCII string
% Using Hershey font 
% Params: str, hershey, scale, spacing, zWrite, zLift, pointsperStroke
% Return: [N×3] matrix in mm- [x, y, z]

    traj = [];
    xOffset = 0; % Start from 0 offset. Add each char offset later

    for c = 1:length(str)
        charData = hershey{str(c)};
        strokes = densifyPaths(charData.stroke, pointsperStroke);
        strokes = scale * strokes;

        % Add z(initially 0s)
        path = [strokes; zeros(1, size(strokes, 2))];

        % Lift pen at stroke breaks (NaNs)
        k = find(isnan(path(1, :)));
        path(:, k) = path(:, k-1);
        path(3, k) = zLift / 1000; % meters

        % Convert to mm and shift in X
        charTraj = path' * 1000;
        charTraj(:,1) = charTraj(:,1) + xOffset;

        traj = [traj; charTraj];
 
        % --- Add lift-up and travel between characters ---
        if c < length(str)
            % Lift pen at end
            liftPos = charTraj(end, :); 
            liftPos(3) = zLift;

            % Move in X to next character(pen lifted)
            movePos = liftPos;
            movePos(1) = movePos(1) + spacing;

            % Put down pen at next start
            downPos = movePos;
            downPos(3) = zWrite;

            traj = [traj; liftPos; movePos; downPos];
        end

        xOffset = xOffset + spacing;
    end
end


function denseStroke = densifyPaths(stroke, pointsperStroke)
% Interpolate and add waypoints inside segments 
% Hopefully smoother
% Params: stroke, pointsperStroke
% Return: denseStroke: 2×M array traj

    denseStroke = [];
    currStroke = [];

    for i = 1:size(stroke, 2) - 1
        p1 = stroke(:, i);
        p2 = stroke(:, i + 1);

        if any(isnan(p1)) || any(isnan(p2))
            % Add current stroke to full output with a NaN separator
            if ~isempty(currStroke)
                denseStroke = [denseStroke, currStroke, [NaN; NaN]];
                currStroke = [];
            end
            continue;
        end

        % Linear interpolation between points
        interpolX = linspace(p1(1), p2(1), pointsperStroke);
        interpolY = linspace(p1(2), p2(2), pointsperStroke);
        interpolSeg = [interpolX; interpolY];

        currStroke = [currStroke, interpolSeg];
    end

    % Add last stroke
    if ~isempty(currStroke)
        denseStroke = [denseStroke, currStroke];
    end
end

function T = transform2D(tx, ty, thetaDeg)
% Returns a Rotation matrix for z only rotation and translation in x & y
    theta = deg2rad(thetaDeg);
    T = [cos(theta), -sin(theta), tx;
         sin(theta),  cos(theta), ty;
         0,           0,          1];
end


function traj = makeMathGridTrajectory(expr, hershey, scale, spacing, zWrite, zLift, pointsperStroke)
% Generates a trajectory of given format: 
% 10
%  2 +
% 12
% Numbers are right-aligned in one column
% Operator appears in a separate column (next to operand 2)
% Params: expr, hershey, scale, spacing, zWrite, zLift, pointsperStroke
% Return: [N×3] matrix in mm- [x, y, z]


    expr = strrep(expr, '=', '');
    resStr = num2str(eval(expr));

    % Regex parsing of I/P string
    tokens = regexp(expr, '(\d+)([\+\-\*])(\d+)', 'tokens');
    if isempty(tokens)
        error('Invalid math expression');
    end

    op1 = tokens{1}{1};
    operator = tokens{1}{2};
    op2 = tokens{1}{3};

    if strcmp(operator, '*')
        operator = 'x';  % replace * with handwritten-style x
    end

    % Determine column width
    maxWidth = max([length(op1), length(op2), length(resStr)]);
    digitColHorizontalSpacing = 30;
    operatorColX = maxWidth * digitColHorizontalSpacing + 5;  % Operator in separate column

    numOfLines = {
        pad(op1, maxWidth, 'left');
        pad(op2, maxWidth, 'left');       % operator
        pad(resStr, maxWidth, 'left')
    };

    operatorLineIndex = 2;  % Place operator with line 2

    traj = [];
    yOffset = 0;

    for i = 1:length(numOfLines)
        line = numOfLines{i};
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

            strokes = densifyPaths(hershey{ch}.stroke, pointsperStroke);
            if ch == '-'
                strokes(1,:) = 0.6 * strokes(1,:);  % squeeze X dimension for minus sign
            end
            strokes = scale * strokes;
            path = [strokes; zeros(1, size(strokes, 2))];

            k = find(isnan(path(1,:)));
            path(:,k) = path(:,k-1);
            path(3,k) = zLift / 1000;

            charTraj = path' * 1000;
            charTraj(:,1) = charTraj(:,1) + (j - 1) * digitColHorizontalSpacing;
            charTraj(:,2) = charTraj(:,2) - yOffset;

            traj = [traj; charTraj];

            % --- Add lift-up and travel between chars ---
            if j < length(line)
                % Lift pen at end
                liftPos = charTraj(end, :); 
                liftPos(3) = zLift;
    
                % Move in X to next character(pen lifted)
                movePos = liftPos;
                movePos(1) = movePos(1) + spacing;
    
                % Pen down at next start
                downPos = movePos;
                downPos(3) = zWrite;
    
                traj = [traj; liftPos; movePos; downPos];
            end
        
            if and(i == 2, j == length(line))
                % Lift pen at end
                liftPos = charTraj(end, :); 
                liftPos(3) = zLift;
    
                traj = [traj; liftPos;];
            end
        
        end

        % Add operator in separate column, but only to line 2
        if i == operatorLineIndex
            ch = operator;
            if ~isempty(hershey{ch})
                strokes = densifyPaths(hershey{ch}.stroke, pointsperStroke);
                strokes = scale * strokes;
                path = [strokes; zeros(1, size(strokes, 2))];

                k = find(isnan(path(1,:)));
                path(:,k) = path(:,k-1);
                path(3,k) = zLift / 1000;

                charTraj = path' * 1000;
                charTraj(:,1) = charTraj(:,1) + operatorColX;
                charTraj(:,2) = charTraj(:,2) - yOffset;

                traj = [traj; charTraj];

                % --- Add lift-up and travel between characters ---
                if j < length(line)
                    % Lift pen at end
                    liftPos = charTraj(end, :); 
                    liftPos(3) = zLift;
        
                    % Move in X to next character (pen lifted)
                    movePos = liftPos;
                    movePos(1) = movePos(1) + spacing;
        
                    % Pen down at next start
                    downPos = movePos;
                    downPos(3) = zWrite;
        
                    traj = [traj; liftPos; movePos; downPos];
                end
            end
        end

        yOffset = yOffset + 50; % Spce between rows
        
        if j == length(line)
            % Lift pen at end
            liftPos = charTraj(end, :); 
            liftPos(3) = zLift;

            traj = [traj; liftPos;];
        end

    end
end


function printMathPreview(expr)
% Console print function showing the expected math result written
    expr = strrep(expr, '=', '');
    resStr = num2str(eval(expr));

    % Regex parse expression
    tokens = regexp(expr, '(\d+)([\+\-\*])(\d+)', 'tokens');
    if isempty(tokens)
        error('Invalid expression');
    end

    op1 = tokens{1}{1};
    operator = tokens{1}{2};
    op2 = tokens{1}{3};

    % Replace '*' with 'x' for writing
    if strcmp(operator, '*')
        operator = 'x';
    end

    % Determine max column width
    maxWidth = max([length(op1), length(op2), length(resStr)]);

    % Format lines with right-alignment
    line1 = pad(op1, maxWidth, 'left');              % operand1
    line2 = pad(op2, maxWidth, 'left');              % operand2
    line3 = pad(resStr, maxWidth, 'left');       % result

    disp("What's being written:");
    % Create rows with operator in separate column
    for i = 1:3
        if i == 1
            fprintf('  %s\n', line1);
        elseif i == 2
            fprintf('  %s %s\n', line2, operator);
        else
            fprintf('  %s\n', line3);
        end
    end
end


clear all; clc;

% Start RVC module
startup_rvc;
load hershey; % Load hershey fonts

% Create RTDE instance
% host = '127.0.0.1';
host = '192.168.0.100'; % REAL ROBOT
port = 30003;
robo = rtde(host, port);

% Writing params
scale = 0.04;
spacing = 30;
% Recco
pointsperStroke = 3;
a = 1.0;
v = 0.5;
r = 0.0035;

% 
% HEIGHTS
% Considering length of pen
zWrite = 21;
zLift = 1.4 * zWrite; % 9mm above writing pos


home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0.00];
startPos = [-588.53, -350, zLift, home(4:6)]; % Start pos of first stroke (mm)

% Move the robot to the home position
disp("-----MOVING TO HOME-----");
robo.movej(home);
%robo.movej(startPos);

path = [];

str = input("What do you want to write(max 10 chars and 5 for math): ", "s");

if contains(str, {'+', '-', '*'}) && str(end) == '=' && ~isempty(regexp(str, '^\d+[\+\-\*]\d+=$', 'once'))
    % CALCULATEEE
    disp("---------------MATH STUFF----------------");
    printMathPreview(str);

    xOffset = input("Enter the x-offset: ");
    xOffset = round(xOffset);
    yOffset = input("Enter the y-offset: ");
    yOffset = round(yOffset);
    yawOffset = input("Enter the yaw-offset: ");
    yawOffset = round(yawOffset);
    
    % Get raw trajectory
    traj = makeMathGridTrajectory(str, hershey, scale, spacing, zWrite, zLift, pointsperStroke);
    
    % Apply transformation
    T = transform2D(xOffset, yOffset, yawOffset+90);
    xy = traj(:,1:2)';
    xyH = [xy; ones(1, size(xy,2))];
    xyTransformed = T * xyH;
    traj(:,1:2) = xyTransformed(1:2,:)';
    
    path = [];
    for i = 1:size(traj,1)
        tcpXYZ = traj(i,1:3) + [startPos(1), startPos(2), zWrite];  % Apply global offset

        if tcpXYZ(3) == zLift + zWrite
            tcpXYZ(3) = tcpXYZ(3) - zWrite;
        elseif tcpXYZ(3) == 2 * zWrite
            tcpXYZ(3) = zLift;
        end

        %disp(tcpXYZ(3));
        if tcpXYZ(3) == zLift
            pose = [tcpXYZ, home(4:6), a, v, 0, 0];
        else
            pose = [tcpXYZ, home(4:6), a, v, 0, r];
        end
        path = [path; pose];
    end

else
    % Write only
    disp("-------------------------WRITE MODE--------------------------");
    
    if length(str) ~= 10 || ~all(ismember(str, ['0':'9', 'a':'z']))
        error('Invalid input. Must be 10 characters from 0–9 and a–z.');
        return;
    end
    
    myTraj = makeTrajFromStr(str, hershey, scale, spacing, zWrite, zLift, pointsperStroke);
    
    disp("Text to write: ");
    disp(str);
    
    xOffset = input("Enter the x-offset: ");
    xOffset = round(xOffset);
    yOffset = input("Enter the y-offset: ");
    yOffset = round(yOffset);
    yawOffset = input("Enter the yaw-offset: ");
    yawOffset = round(yawOffset);
    
    % 2D transformation
    T = transform2D(xOffset, yOffset, yawOffset+90);
    
    % Apply to X-Y, leave Z unchanged
    xy = myTraj(:,1:2)';
    xyH = [xy; ones(1, size(xy,2))];        % Convert to homogeneous (3×N)
    xyTransformed = T * xyH;
    % Update trajectory
    myTraj(:,1:2) = xyTransformed(1:2,:)';
    
    for i = 1:size(myTraj,1)
        tcpXYZ = myTraj(i,1:3) + [startPos(1), startPos(2), zWrite];  % Apply global offset
        
        if tcpXYZ(3) == zLift + zWrite
            tcpXYZ(3) = tcpXYZ(3) - zWrite;
        elseif tcpXYZ(3) == 2 * zWrite
            tcpXYZ(3) = zLift;
        end
        
        %disp(tcpXYZ(3));
        if tcpXYZ(3) == zLift
            pose = [tcpXYZ, home(4:6), a, v, 0, 0];
        else
            pose = [tcpXYZ, home(4:6), a, v, 0, r];
        end
        path = [path; pose];
    end
end

% Execute the movement
disp("-----STARTING ON TRAJECTORY-----");
poses = robo.movel(path);
robo.movej(home);

robo.drawPath(poses);  % Graph out TCP

% Close RTDE instance
robo.close();
disp("");
disp("-----------PROGRAM COMPLETE-----------");