global speed;
speed = 75;
global key;
doKeyInput = false;

% reconnect, configure, and initialize brick
brick = initBrick(brick);

% set first placement variables
clock = tic; % start clock

if doKeyInput
    InitKeyboard();
    manualMode(brick);
end

colorFunction(brick); % check color before moving
time2 = toc(clock); % save time
lastDist = brick.UltrasonicDist(2); % get distance from sensor
moveForward(brick); % start moving

% main loop starts here
while true

    loopStartTime = toc(clock);
    
    distanceReading = brick.UltrasonicDist(2);
    if (distanceReading < 0) || (distanceReading >= 255)
        disp("ERROR - Distance Sensor Reading: " + distanceReading);
        continue;
    end

    % this is a whole thing lmao
    if colorFunction(brick)
        % save new distance to wall after turn
        lastDist = brick.UltrasonicDist(2);
        % continue forward
        moveForward(brick);
        continue;
    end

    % if front touch sensor is hitting wall
    if isTouching(brick)
        % stop robot
        brick.StopAllMotors(1);
        pause(0.2);
        % move backward a little
        nudgeBack(brick);
        pause(0.2);
        % turn right automatically
        turnAuto(brick, clock, 'right');
        pause(0.2);

        % save new distance to wall after turn
        lastDist = brick.UltrasonicDist(2);
        % continue forward
        moveForward(brick);
        continue;
    end
    
    % if there is a hole in the wall
    if distanceReading > 38
        pause(0.1);
        % wait a little bit and double check to be certain
        % the sensor isn't lying/bugging
        if brick.UltrasonicDist(2) > 38
            pause(0.6);
            brick.StopAllMotors(1);
            pause(0.2);
            turnAuto(brick, clock, 'left');
            pause(0.2);

            % save new distance to wall after turn
            lastDist = brick.UltrasonicDist(2);
            % continue forward
            moveForward(brick);
            continue;
        end
    end

    % note to possibly add code to make robot drift toward center of path
    % only add if robot get caught too close to wall

    % apply adjustment so robot does not drift into wall
    % only run once every 0.25 seconds for faster runtime
    timeDifference = toc(clock) - time2;
    if timeDifference > 0.25
        % get the drift rate of change to tell which direction it is
        % drifting in
        driftROC = (brick.UltrasonicDist(2) - lastDist) / timeDifference;
        disp("DriftROC: " + driftROC);
        if driftROC > 0
            disp("Drifting Right");
            brick.MoveMotor('A', -speed * 0.9);
        else
            disp("Drifting Left");
            brick.MoveMotor('D', -speed * 0.9);
        end
        pause(0.25 * abs(driftROC));
        time2 = toc(clock);
    end

    loopEndTime = toc(clock);
    disp("Main Loop Execution Time: " + (loopEndTime - loopStartTime) + " seconds");
    pause(0.01)
end

% base movement functions
function moveForward(brick)
    global speed;
    brick.MoveMotor('A', -speed);
    brick.MoveMotor('D', -speed);
end

function moveBackward(brick)
    global speed;
    brick.MoveMotor('A', speed);
    brick.MoveMotor('D', speed);
end

% manual turning functions
function turnRight(brick)
    brick.MoveMotor('A', -100);
    brick.MoveMotor('D', 100);
end

function turnLeft(brick)
    brick.MoveMotor('A', 100);
    brick.MoveMotor('D', -100);
end

% slightly move backwards after turns
function nudgeBack(brick)
    % apparently motor A was 65% weaker than motor D at the time
    % we should double check this
    brick.MoveMotor('A', 100 * 0.65);
    brick.MoveMotor('D', 100);
    pause(0.35)
    brick.StopAllMotors(1);
end

% crunched turnAutoLeft() and turnAutoRight() into a single function
function turnAuto(brick, clock, direction)
    % the left and right motor powers are not -100 and 100 for turning
    % because through some testing it seemed about 60% power on the inner
    % motor created a smoother, more stable turn
    % we can futher test this however as our robot design has changed
    switch (direction)
        case 'right'
            APower = -100;
            DPower = 60;
        case 'left'
            APower = 60;
            DPower = -100;
    end

    % inside a try/catch statement in case of error, robot will continue to
    % function
    try
        startAngle = brick.GyroAngle(1);
        brick.MoveMotor('A', APower);
        brick.MoveMotor('D', DPower);

        time1 = toc(clock);

        currentAngle = startAngle - brick.GyroAngle(1); 
        while currentAngle < 90
            % Apply gradual slowdown as we approach goal, results in a more
            % accurate final turn

            if angleDiff > 45 % only apply when halfway through turn (full speed for first half)
                temp = (1 - (angleDiff / 90))*2; % transforms angle range (45-90 degrees) to this range (1.0-0.5)
                brick.MoveMotor('A', APower * temp);
                brick.MoveMotor('D', DPower * temp);
            end

            % this is the part that checks if robot is stuck during turn
            % if change in time since last execution is > than 1 second
            % speeds up code by checking less often
            timeDiff2 = toc(clock) - time1;
            if timeDiff2 > 1
                angle2 = brick.GyroAngle(1);
                if abs(angle1 - angle2) < 1
                    disp("Possibly Stuck, Attempting Unstuck")
                    brick.StopAllMotors(0);
                    pause(0.1)
                    brick.MoveMotor('A', APower);
                    brick.MoveMotor('D', DPower);
                end
                angle1 = angle2;
                time1 = toc(clock);
            end
        end
        % turn is now completed
        brick.StopAllMotors(1);
    catch 
        disp("ERROR IN turnAuto() Function");
    end
end

% returns variable result, which is true is touch sensor is pushed
function result = isTouching(brick)
    result = false;
    try
        if brick.TouchPressed(3) == 1
            result = true;
        end
    catch
        disp("ERROR IN isTouching() Function");
    end
end

function success = colorFunction(brick)
    success = false;

    function result = getColor(brick)
        result = 'none';
        try

            % fix color values/comparisons

            color_rgb = brick.ColorRGB(4);
            if color_rgb(1) > 100
                result = 'red';
            elseif color_rgb(3) > 100
                result = 'blue';
            elseif color_rgb(2) > 100
                result = 'green';
            end
        catch
            disp("ERROR IN getColor() Function");
        end
    end

    function moveUntilNotOnColor(brick, color)
        moveForward(brick);
        while getColor(brick) == color
            pause(0.01);
        end
    end

    switch (getColor(brick))
        case 'red'
            disp("Stop Detected")
            brick.StopAllMotors(1);
            pause(3);
            moveUntilNotOnColor(brick, 'red')
            success = true;
        case 'blue'
            disp("Blue Detected")
            brick.StopAllMotors(1);

            brick.playTone(200, 0.3);
            pause(0.3);
            brick.playTone(200, 0.3);

            moveUntilNotOnColor(brick, 'blue');
            success = true;
        case 'green'
            disp("Green Detected")
            brick.StopAllMotors(1);

            brick.playTone(200, 0.3);
            pause(0.3);
            brick.playTone(200, 0.3);
            pause(0.3);
            brick.playTone(200, 0.3);

            moveUntilNotOnColor(brick, 'green');
            success = true;
    end
end

function brick = initBrick(brick)
    clc;
    clear color_rgb;
    clear distance;
    clear lastDist;
    clear timeDifference;
    clear clock;
    
    % reconnect brick at start because sensors go wacky a lot
    try
        delete(brick);
        brick = ConnectBrick('THEBOI');
    catch
        disp("Could not reconnect robot :( Now exiting, please connect manually");
        pause(3)
        return
    end
    disp(brick);
    pause(0.5);
    
    % establish modes and ensure sensors are all functioning properly
    brick.GyroCalibrate(1);
    brick.StopAllMotors(0);
    brick.SetColorMode(4, 4);
    disp("Touch Reading: " + brick.TouchPressed(3));
    disp("Current Angle: " + brick.GyroAngle(1));
    pause(1);
end

function manualMode(brick)
    global key;
    doKeyInput = true;
    while doKeyInput
        switch (key) 
            case 'w'
                moveForward(brick);
            case 's'
                moveBackward(brick);
            case 'a'
                turnLeft(brick);
            case 'd'
                turnRight(brick);
            case 'f'
                brick.MoveMotor('B', 30);
            case 'g'
                brick.MoveMotor('B', -30);
            case 'q'
                disp("QUITING KEY INPUT AND ENTERING AUTO");
                doKeyInput = false;
            otherwise
                brick.StopAllMotors(1);
        end
    end
end