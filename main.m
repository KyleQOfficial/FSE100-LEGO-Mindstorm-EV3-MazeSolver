global firstAngle;
global speed;
speed = 75;
global key;
doKeyInput = false;

% TODO LIST
% minor color detection improvements should be made
% slight tweeking to power curves and correction in general

% reconnect, configure, and initialize brick
brick = initBrick(brick);

% set first placement variables
clock = tic; % start clock

if doKeyInput
    InitKeyboard();
    manualMode(brick);
end

colorFunction(brick, clock); % check color before moving
time = toc(clock); % get current time
firstAngle = brick.GyroAngle(1); % get current angle
moveForward(brick); % start moving

% main loop starts here
while true

    loopStartTime = toc(clock);
    
    if touchFunction(brick, clock, firstAngle)
        firstAngle = brick.GyroAngle(1);
        moveForward(brick);
    end

    distanceReading = brick.UltrasonicDist(2);
    if (distanceReading < 0) || (distanceReading >= 255)
        disp("ERROR - Distance Sensor Reading: " + distanceReading);
        stopRobot(brick, speed);
        nudgeBack(brick);
        continue;
    end

    % colorFunction() checks and does all the color stuff,
    % if a color is found, then it returns true and runs the following code
    if colorFunction(brick, clock)
        firstAngle = brick.GyroAngle(1);
        moveForward(brick);
    end

    % if there is a hole in the wall
    if distanceReading > 38
        pause(0.1);
        % wait a little bit and double check to be certain
        % the sensor isn't lying/bugging
        if brick.UltrasonicDist(2) > 38
            pause(0.75); % TIME
            stopRobot(brick, speed);
            pause(0.2);
            turnAuto(brick, clock, 'left', firstAngle);
            pause(0.2);
            correctWithWall(brick);

            firstAngle = brick.GyroAngle(1);
            moveForward(brick);
            continue;
        end
    end

    % correction based on gyro angle
    if toc(clock) - time > 0.25
        
        angleDifference = firstAngle - brick.GyroAngle(1);
        disp("Angle Diff: " + angleDifference);
    
        if  angleDifference < 0
            disp("Drifting Right");
            brick.MoveMotor('A', -(speed-2));
        elseif angleDifference > 0
            disp("Drifting Left");
            brick.MoveMotor('D', -(speed-2));
        else
            continue;
        end

        time = toc(clock);
    end
    
    loopEndTime = toc(clock);
    %disp("Main Loop Execution Time: " + (loopEndTime - loopStartTime) + " seconds");
    pause(0.001)
end

% base movement functions
function moveForward(brick)
    global speed;
    brick.MoveMotor('A', -speed);
    brick.MoveMotor('D', -speed);
end

function moveUltimate(brick, clock, speed, driveTime)
    timeDiff = 0;

    brick.MoveMotor('D', speed);
    brick.MoveMotor('A', speed);

    startTime = toc(clock);

    while timeDiff < driveTime
        colorFunction(brick, clock);
        timeDiff = toc(clock) - startTime;
        pause(0.01);
    end
    stopRobot(brick, speed);
end

function stopRobot(brick, lspeed)
    %brick.playTone(100, 500, 50);
    if lspeed > 0
        while lspeed > 0
            lspeed = max(lspeed - 15, 0);
            brick.MoveMotor('A', lspeed);
            brick.MoveMotor('D', lspeed);
            pause(0.001);
        end
    else
        while lspeed < 0
            lspeed = min(lspeed + 15, 0);
            brick.MoveMotor('A', lspeed);
            brick.MoveMotor('D', lspeed);
            pause(0.001);
        end
    end

    brick.StopAllMotors(1);
    pause(0.1);
    brick.StopAllMotors(0);
end

function moveBackward(brick)
    global speed;
    brick.MoveMotor('A', speed);
    brick.MoveMotor('D', speed);
end

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
    global speed;
    speed = 100;
    brick.MoveMotor('A', speed * 0.95);
    brick.MoveMotor('D', speed);
    pause(0.2); % TIME
    stopRobot(brick, speed);
    pause(0.2);
    speed = 75;
end

function success = touchFunction(brick, clock, firstAngle)
    success = false;
    if brick.TouchPressed(3) == 1
        brick.StopAllMotors(1);
        pause(0.1);
        % move backward a little
        nudgeBack(brick);
        % turn right automatically
        turnAuto(brick, clock, 'right', firstAngle);
        pause(0.2);
        correctWithWall(brick);

        success = true;
    end
end

% crunched turnAutoLeft() and turnAutoRight() into a single function
function turnAuto(brick, clock, direction, startAngle)
    disp("Started turning " + direction);

    % assign variables/values according to turn direction
    switch (direction)
        case 'right'
            APower = -75;
            DPower = 75;
        case 'left'
            APower = 75;
            DPower = -75;
    end
    
    angle1 = startAngle;
    time1 = toc(clock);
    currentAngle = abs(angle1 - brick.GyroAngle(1));

    while currentAngle < 85
        disp(currentAngle);

        % Apply gradual slowdown as we approach goal, prevents overshooting
        temp = max(1 - (currentAngle / 90), 0.2); % transforms angle range (0-90 degrees) to this range (1.0-0.2)

        brick.MoveMotor('A', APower * temp);
        brick.MoveMotor('D', DPower * temp);

        % this is the part that checks if robot is stuck during turn
        % if change in time since last execution is > than 1 second
        % speeds up code by checking less often
        timeDiff = toc(clock) - time1;
        if timeDiff > 1
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
    
    brick.StopAllMotors(1);
    pause(0.1);
    brick.StopAllMotors(0);
    pause(0.1);
    disp("Finished turning");
    
    % only need to do a few things after LEFT turn only
    if strcmp(direction, 'left')
        moveUltimate(brick, clock, -50, 2.5); % FIX TIMES
    end

end

function success = colorFunction(brick, clock)
    global speed;
    global firstAngle;
    success = false;

    function result = getColor(brick)
        result = 'none';
        try

            % fix color values/comparisons

            color_rgb = brick.ColorCode(4);
            switch (color_rgb)
                case 2
                    speed = 35;
                    result = 'blue';
                case 3
                    speed = 35;
                    result = 'green';
                case 5
                    speed = 35;
                    result = 'red';
            end
        catch
            disp("ERROR IN getColor() Function");
        end
    end

    function moveUntilNotOnColor(brick, color, clock)
        moveForward(brick);
        currentColor = getColor(brick);

        while strcmp(currentColor, color)
            if touchFunction(brick, clock, firstAngle)
                firstAngle = brick.GyroAngle(1);
                moveForward(brick);
            end
            currentColor = getColor(brick);
            pause(0.01);
        end
    end

    color = getColor(brick);
    switch (color)
        case 'red'
            disp("Stop Detected")
            brick.StopAllMotors(1);
            pause(3);
            moveUntilNotOnColor(brick, 'red', clock);
            moveForward(brick);
            pause(0.5);
            brick.StopAllMotors(0);
            pause(0.5);
            speed = 75;
            success = true;
        case 'blue'
            disp("Blue Detected")
            stopRobot(brick, speed);

            brick.beep(1);
            pause(1);
            brick.beep(1);

            moveUntilNotOnColor(brick, 'blue', clock);
            speed = 75;
            success = true;
        case 'green'
            disp("Green Detected")
            brick.StopAllMotors(1);

            brick.beep(1);
            pause(1);
            brick.beep(1);
            pause(1);
            brick.beep(1);

            moveUntilNotOnColor(brick, 'green', clock);
            speed = 75;
            success = true;
    end
end

function brick = initBrick(brick)
    clc;
    % add variables to clear
    
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
    brick.SetColorMode(4, 2);
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

% this function will be run after turning to ensure robot is mostly
% straight
function correctWithWall(brick)
    global speed;
    % function to get readings because it has to double check and its kinda
    % long, only allowed to take extra time here checking reading because
    % robot is stopped
    function distance = getDistance(brick, lspeed)
        stopRobot(brick, lspeed);
        pause(0.25);
        distance = brick.UltrasonicDist(2);
        pause(0.25);
        if abs(distance - brick.UltrasonicDist(2)) > 1
            disp("ERROR BETWEEN READINGS OFF BY MORE THAN ERROR ALLOWED");
        end
    end

    count = 1;
    firstTurn = 'none';

    while true

        if count > 10
            break;
        end

        % get first reading
        firstReading = getDistance(brick, 0);
        % move forward
        brick.MoveMotor('A', -50);
        brick.MoveMotor('D', -50);
        pause(0.2);
        % get second reading
        secondReading = getDistance(brick, -50);
        
        % compare the two readings
        difference = firstReading - secondReading;
        
        % if the difference is greater than 10 then something most likely
        % went wrong because the robot does not move enough for it to be
        % this high of a value
        if abs(difference) > 10
            disp("BAD DISTANCE READING");
            continue;
        end

        % apply proper power to turn robot in proper direction,
        % low starting power that decreases every correction
        power = max(35 - (count*4), 10);
        disp("Power: " + power);

        if difference < 0
            if strcmp(firstTurn, 'none')
                firstTurn = 'left';
            end
            brick.MoveMotor('A', power);
            brick.MoveMotor('D', -power);
            if strcmp(firstTurn, 'right')
                disp("APPEARS STRAIGHT");
                brick.beep(1);
                break;
            end
        else
            if strcmp(firstTurn, 'none')
                firstTurn = 'right';
            end
            brick.MoveMotor('A', -power);
            brick.MoveMotor('D', power);
            if strcmp(firstTurn, 'left')
                disp("APPEARS STRAIGHT");
                brick.beep(1);
                break;
            end
        end
        
        pause(0.1);
        brick.StopAllMotors(0);
        pause(0.2); % CHANGE - CAN LOWER

        brick.MoveMotor('A', 50);
        brick.MoveMotor('D', 50);
        pause(0.2);

        firstReading = getDistance(brick, 50);

        % compare the two readings
        difference = firstReading - secondReading;
        
        % if the difference is greater than 10 then something most likely
        % went wrong because the robot does not move enough for it to be
        % this high of a value
        if abs(difference) > 10
            disp("BAD DISTANCE READING");
            continue;
        end

        % apply proper power to turn robot in proper direction,
        % low starting power that decreases every correction

        if difference > 0
            if strcmp(firstTurn, 'none')
                firstTurn = 'left';
            end
            brick.MoveMotor('A', power);
            brick.MoveMotor('D', -power);
            if strcmp(firstTurn, 'right')
                disp("APPEARS STRAIGHT");
                brick.beep(1);
                break;
            end
        else
            if strcmp(firstTurn, 'none')
                firstTurn = 'right';
            end
            brick.MoveMotor('A', -power);
            brick.MoveMotor('D', power);
            if strcmp(firstTurn, 'left')
                disp("APPEARS STRAIGHT");
                brick.beep(1);
                break;
            end
        end
        
        pause(0.1);
        brick.StopAllMotors(0);
        pause(0.5); % CHANGE - CAN LOWER

        brick.StopAllMotors(0);
        pause(0.2);

        disp("" + count + ": " + difference);
        count = count + 1;

    end
end
