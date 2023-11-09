global firstAngle;
global speed;
global key;
speed = 75;
doKeyInput = true;

% TODO LIST
% implement stuck check?
% code in general should be refactored (made faster, improved functions, tidy up code) <-- last step
% experiment with lowering all pauses

brick = initBrick(brick); % reconnect, configure, and initialize brick/sensors
clock = tic; % start clock

if doKeyInput
    InitKeyboard();
    manualMode(brick);
end

colorFunction(brick, clock); % check color before moving
time = toc(clock); % get current time
firstAngle = brick.GyroAngle(1); % get starting angle
moveForward(brick); % start moving

stuckTotal = 0;
stuckCount = 0;

% main loop starts here
while true

    loopStartTime = toc(clock);
    
    if touchFunction(brick, clock, firstAngle)
        firstAngle = brick.GyroAngle(1);
        moveForward(brick);
    end

    %distanceReading = brick.UltrasonicDist(2);
    if (distanceReading < 0) || (distanceReading >= 255)
        disp("ERROR - Distance Sensor Reading: " + distanceReading);
        stopRobot(brick, speed);
        nudgeBack(brick);
        pause(5);
        continue;
    end

    % colorFunction() checks and does all the color stuff,
    % if a color is found, then it returns true and runs the following code
    if colorFunction(brick, clock)
        firstAngle = brick.GyroAngle(1);
        moveForward(brick);
    end
    
    if holeFunction(brick, clock, distanceReading)
        firstAngle = brick.GyroAngle(1);
        moveForward(brick);
    end

    % correction based on gyro angle
    if toc(clock) - time > 0.25
        
        angleDifference = firstAngle - brick.GyroAngle(1);
        %disp("Angle Diff: " + angleDifference);
    
        if  angleDifference < 0
            %disp("Drifting Right");
            brick.MoveMotor('A', -(speed-2));
        elseif angleDifference > 0
            %disp("Drifting Left");
            brick.MoveMotor('D', -(speed-2));
        else
            continue;
        end
        
        disp(brick.GyroRate(1));
        temp = abs(brick.GyroRate(1));
        disp(temp);
        if temp < 5
            %stuckTotal = stuckTotal + temp;
            stuckCount = stuckCount + 1;
        else
            stuckCount = 0;
        end

        time = toc(clock);
    end

    if stuckCount > 10
        disp("STUCK?");
        brick.StopAllMotors(1);
        pause(3);
        nudgeBack(brick);
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

% (1) experimental braking using gradual slowdown at first, (2) then applying full
% break force, (3) then relaxing motors so they are not under tension.
% helps reduce "jumping" when instantly applying full break force
function stopRobot(brick, lspeed)
    %brick.playTone(100, 500, 50);
    % (1)
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
    % \(1)

    brick.StopAllMotors(1); % (2)
    pause(0.1);
    brick.StopAllMotors(0); % (3)
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

% moves the robot backwards slightly
function nudgeBack(brick)
    brick.MoveMotor('A', 100 * 0.95); % less power on left motor because it seemed stronger at the time
    brick.MoveMotor('D', 100);
    pause(0.2); % TIME
    stopRobot(brick, 100);
    pause(0.2);
end

% (1) if touch sensor is detected as pressed, (2) instantly stop robot
% because it has already slammed into a wall, (3) nudgeBack away from wall,
% (4) complete automatic right turn, (5) correct with wall
function success = touchFunction(brick, clock, firstAngle)
    success = false;
    if brick.TouchPressed(3) == 1 % (1)
        brick.StopAllMotors(1); % (2)
        pause(0.1); % waiting for breaking to take full effect
        nudgeBack(brick); % (3)
        turnAuto(brick, clock, 'right', firstAngle); % (4)
        pause(0.2);
        correctWithWall(brick); % (5)

        success = true;
    end
end

% (1) at any point, if the distance is greater than ~38cm, then there must
% be a hole in the wall and we must explore it, (2) double check, 
function success = holeFunction(brick, clock, distanceReading)
    global firstAngle;
    global speed;
    success = false;
    
    if distanceReading > 38 % (1)
        pause(0.1);
        % (2) wait a little bit and double check to be certain
        % the sensor isn't lying/bugging
        % not using distanceReading because we need to request a new value
        % directly from the robot again to double check it
        if brick.UltrasonicDist(2) > 38
            % wait a while longer so it is oriented properly to make turn
            pause(0.75); % TIME
            stopRobot(brick, speed);
            pause(0.2);
            turnAuto(brick, clock, 'left', firstAngle);
            pause(0.2);
            correctWithWall(brick);
            success = true;
        end
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

    function color = getColor(brick)
        color = 0;
        try
            color = brick.ColorCode(4);
        catch
            disp("ERROR IN getColor() Function");
        end
    end

    function moveUntilNotOnColor(brick)
        moveForward(brick);
        color = getColor(brick);

        while color == 2 || color == 3 || color == 5 || color == 4
            if touchFunction(brick, clock, firstAngle)
                firstAngle = brick.GyroAngle(1);
                moveForward(brick);
            end
            color = getColor(brick);
            pause(0.001);
        end
        stopRobot(brick, speed);
    end

    color = getColor(brick);
    disp("Color Code: " + color)
    if color == 2 || color == 3 || color == 5 || color == 4
        speed = 35;
        stopRobot(brick, speed);
        pause(3);
        moveUntilNotOnColor(brick);
        moveUltimate(brick, clock, -35, 1);
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
                brick.MoveMotor('B', 35);
            case 'g'
                brick.MoveMotor('B', -35);
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
    % robot is already stopped
    function distance = getDistance(lspeed)
        stopRobot(brick, lspeed);
        pause(0.25);
        distance = brick.UltrasonicDist(2);
        pause(0.25);
        if abs(distance - brick.UltrasonicDist(2)) > 1
            disp("ERROR BETWEEN READINGS OFF BY MORE THAN ERROR ALLOWED");
        end
    end

    function leftCorrect()
        if strcmp(firstTurn, 'none')
            firstTurn = 'left';
        end
        brick.MoveMotor('A', power);
        brick.MoveMotor('D', -power);
    end

    function rightCorrect()
        if strcmp(firstTurn, 'none')
            firstTurn = 'right';
        end
        brick.MoveMotor('A', -power);
        brick.MoveMotor('D', power);
    end

    count = 1;
    firstTurn = 'none';

    while true

        if count > 6
            break;
        end

        % get first reading
        firstReading = getDistance(0);
        % move forward
        brick.MoveMotor('A', -29);
        brick.MoveMotor('D', -30);
        pause(0.5);
        % get second reading
        secondReading = getDistance(30);
        
        % compare the two readings
        difference = firstReading - secondReading;
        
        % if the difference is greater than 10 then something most likely
        % went wrong because the robot does not move enough for it to be
        % this high of a value
        if abs(difference) > 5
            disp("BAD DISTANCE READING");
            continue;
        end

        % calculate proper power to turn robot in proper direction,
        % lower starting power that decreases every correction
        power = max(35 - (count*5), 15);
        disp("Power: " + power);

        if difference < 0
            leftCorrect();
        else
            rightCorrect();
        end
        
        pause(0.1);
        brick.StopAllMotors(0);
        pause(0.2);

        firstReading = getDistance(0);

        brick.MoveMotor('A', 29);
        brick.MoveMotor('D', 30);
        pause(0.5);

        secondReading = getDistance(30);

        difference = firstReading - secondReading;
        
        if abs(difference) > 5
            disp("BAD DISTANCE READING");
            continue;
        end

        if difference > 0
            leftCorrect();
        else
            rightCorrect();
        end
        
        pause(0.1);
        brick.StopAllMotors(0);
        pause(0.2);

        disp("Difference: " + difference);
        count = count + 1;

    end
end
