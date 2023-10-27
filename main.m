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
    end

    % colorFunction() checks and does all the color stuff,
    % if a color is found, then it returns true and runs the following code
    if colorFunction(brick)
        % save new distance to wall after turn
        lastDist = brick.UltrasonicDist(2);
        % continue forward
        moveForward(brick);
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
        end
    end

    % OLD CORRECTION
    % note to possibly add code to make robot drift toward center of path
    % only add if robot get caught too close to wall

    % apply adjustment so robot does not drift into wall
    % only run once every 0.25 seconds for faster runtime
    % disp("Distance: " + brick.UltrasonicDist(2));
    % if brick.UltrasonicDist(2) < 38
    %     timeDifference = toc(clock) - time2;
    %     if timeDifference > 0.25
    %         % get the drift rate of change to tell which direction it is
    %         % drifting in
    %         driftROC = (brick.UltrasonicDist(2) - lastDist) / timeDifference;
    %         disp("DriftROC: " + driftROC);
    %         if driftROC > 0
    %             disp("Drifting Right");
    %             brick.MoveMotor('A', -speed * 0.9);
    %         else
    %             disp("Drifting Left");
    %             brick.MoveMotor('D', -speed * 0.9);
    %         end
    %         pause(0.2);
    %         time2 = toc(clock);
    %         lastDist = brick.UltrasonicDist(2);
    %     end
    % end

    loopEndTime = toc(clock);
    %disp("Main Loop Execution Time: " + (loopEndTime - loopStartTime) + " seconds");
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
    brick.MoveMotor('A', 100);
    brick.MoveMotor('D', 100);
    pause(0.35)
    brick.StopAllMotors(1);
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

% crunched turnAutoLeft() and turnAutoRight() into a single function
function turnAuto(brick, clock, direction)
    disp("Started turning " + direction);

    % assign variables/values according to turn direction
    switch (direction)
        case 'right'
            APower = -75;
            DPower = 75;
            timeSleep = 0;
        case 'left'
            APower = 75;
            DPower = -75;
            timeSleep = 1.2;
    end

    angle1 = brick.GyroAngle(1);
    time1 = toc(clock);
    currentAngle = abs(angle1 - brick.GyroAngle(1)); 

    while currentAngle < 90
        % Apply gradual slowdown as we approach goal, prevents overshooting
        temp = max(1 - (currentAngle / 90), 0.25); % transforms angle range (0-90 degrees) to this range (1.0-0.25)
        disp(temp);

        brick.MoveMotor('A', APower * temp);
        brick.MoveMotor('D', DPower * temp);

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

        currentAngle = abs(startAngle - brick.GyroAngle(1)); 
    end

    brick.StopAllMotors(0);
    pause(0.2);
    disp("Finished turning");

    moveForward(brick);
    pause(timeSleep);
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
        temp = getColor(brick);
        disp(temp);
        while strcmp(temp, color)
            temp = getColor(brick);
            pause(0.01);
        end
    end

    switch (getColor(brick))
        case 'red'
            disp("Stop Detected")
            brick.StopAllMotors(1);
            pause(3);
            moveUntilNotOnColor(brick, 'red');
            success = true;
        case 'blue'
            disp("Blue Detected")
            brick.StopAllMotors(1);

            brick.beep(1);
            pause(1);
            brick.beep(1);

            moveUntilNotOnColor(brick, 'blue');
            success = true;
        case 'green'
            disp("Green Detected")
            brick.StopAllMotors(1);

            brick.beep(1);
            pause(1);
            brick.beep(1);
            pause(1);
            brick.beep(1);

            moveUntilNotOnColor(brick, 'green');
            success = true;
    end
end

function brick = initBrick(brick)
    clc;
    clear color_rgb;
    clear distanceReading;
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
                brick.MoveMotor('B', 20);
            case 'g'
                brick.MoveMotor('B', -20);
            case 'q'
                disp("QUITING KEY INPUT AND ENTERING AUTO");
                doKeyInput = false;
            otherwise
                brick.StopAllMotors(1);
        end
    end
end