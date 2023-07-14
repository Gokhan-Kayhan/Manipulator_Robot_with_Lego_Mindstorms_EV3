clear
clc

lego = legoev3('usb'); % connecting through USB

motorA = motor(lego,'A');   % create handle for motor A
motorB = motor(lego,'B');   % create handle for motor B
motorC = motor(lego,'C');   % create handle for motor C

touchC = touchSensor(lego,1);   %connecting to touch sensor of motor C
touchB = touchSensor(lego,3);   %connecting to touch sensor of motor B

sonicSensor = sonicSensor(lego,2);  %connection to ultrasonic sensor


task4(motorA, motorB,touchB,motorC,touchC)
task5(motorA, motorB,motorC,touchB,sonicSensor)
task6(motorA,motorB,motorC,touchB,touchC,sonicSensor)
task7(motorA,motorB,motorC,touchB,touchC,sonicSensor)
task8(motorA,motorB,motorC,touchB,touchC,sonicSensor)
task9(motorA,motorB,motorC,touchB,touchC,sonicSensor)
task10(motorA,motorB,motorC,touchB,sonicSensor)
 

function armUp(motorB,touchB)  %function to raise the grapp up until the limit (toching touch sensor B)
    pause(0.5)  %pause before movement
    motorB.Speed=-30;   %setting speed of Motor B -30
    start(motorB);  %starting motor B
    touch = readTouch(touchB);  %reading value of touch sensor B
    while touch == 0    %waiting for arm to reach limit sensor
        touch = readTouch(touchB);  
    end
    resetRotation(motorB);  %resetting encoder value of motor B to 0
    motorB.Speed = 0;   %stopping motor
end
function resetGripper(motorA)  %function to reset gripper to open position
    motorA.Speed = -10;     %setting speed of motor A to -10
    start(motorA);  %starting motor A
    pause(1)    %waiting 1 second
    motorA.Speed = 0;   %stopping motor A
    resetRotation(motorA);  %resetting encoder of Motor A
    rA=readRotation(motorA);    %reading walue of encoder of motor A
    while rA < 90   %opening gripper
       motorA.Speed = 1;
       rA = readRotation(motorA);
    end
    motorA.Speed = 0;   %stopping moor A
end


function theta = measureHeight(sonicSensor) %function to find theta through inverse kinematics
    pause(1)    %waiting 1 second for stability
    x = readDistance(sonicSensor)*1000;     %reading distance between ground and the sensor
    theta = 5*(rad2deg(atan((236-x-67.17)/sqrt(34225-(236-x-67.17)^2)))+45)*1; 
end
function moveDownToPlatform(motorB,theta, sonicSensor)     %function to move the gripper to the platform
    resetRotation(motorB);      %resetting encoder of motor B
    rB=readRotation(motorB);    %reading value of encoder of motor B
    while rB<(450-theta)*0.75    %running motor B until it goes to 75% of the way
        motorB.Speed=10;    %setting motor B speed to 10
        rB=readRotation(motorB);    %reading value of encoder of motor B
    end
    motorB.Speed=0;
    pause(1)
    theta = measureHeight(sonicSensor)  %measuring height  again
    resetRotation(motorB);      %resetting encoder of motor B
    rB=readRotation(motorB); 
    while rB<(450-theta-20)    %running motor B until it reaches platform
        motorB.Speed=10;    %setting motor B speed to 10
        rB=readRotation(motorB);    %reading value of encoder of motor B
    end
    motorB.Speed=0;     %stopping motor B
end
function closeGripper(motorA)   %This function closes the gripper
    pause(0.5)
    rA = readRotation(motorA);
    while rA > 10
        motorA.Speed = -10;
        rA = readRotation(motorA);
    end
motorA.Speed = 0;
end
function openGripper(motorA)    %This function opens the gripper
    pause(0.5)
    rA = readRotation(motorA);
    while rA < 90
        motorA.Speed = 10;
        rA = readRotation(motorA);
    end
motorA.Speed = 0;
end

function catchBall(motorA,motorB,touchB,sonicSensor)    %we called 4 other functions to go to the platform, catch the ball and go up
    theta = measureHeight(sonicSensor);
    moveDownToPlatform(motorB,theta,sonicSensor);
    closeGripper(motorA);
    armUp(motorB,touchB);
end
function releaseBall(motorA,motorB,touchB,sonicSensor) %%we called 4 other functions to go to the platform, release the ball and go up
    theta = measureHeight(sonicSensor);
    moveDownToPlatform(motorB,theta,sonicSensor);
    openGripper(motorA);
    armUp(motorB,touchB);
end

function rotateCCW(motorC,degree)   %this function rotates motor C until given degree in counter clock with using P controller
    rotation = readRotation(motorC);
    while rotation > degree
        motorC.Speed = -10-(-degree+rotation)/8;    %adding P controller
        if motorC.Speed<-30    %limiting controller value
            motorC.Speed = -30;
        end
        rotation = readRotation(motorC);
    end
    motorC.Speed = 0;
    readRotation(motorC)
end
function rotateCW(motorC,degree)    %this function rotates motor C until given degree in clock with using P controller
    rotation = readRotation(motorC);
    while rotation < degree
        motorC.Speed = 10+(degree-rotation)/8   %%adding P controller
        if motorC.Speed>30  %%limiting controller value
            motorC.Speed = 30;
        end
        rotation = readRotation(motorC);
    end
    motorC.Speed = 0;
    readRotation(motorC)
end

function goToA(motorC,touchC)
    motorC.Speed = 30;
    start(motorC)
    touch = readTouch(touchC);
    while touch == 0
         touch = readTouch(touchC);
    end
    resetRotation(motorC);
    motorC.Speed = 0;
end
function goToBccw(motorC)
    pause(0.5)
    rotateCCW(motorC,-320)
end
function goToBcw(motorC)
    pause(0.5)
    rotateCW(motorC,-300)
end
function goToC(motorC)
    pause(0.5)
    rotateCCW(motorC,-595)
end

function task4(motorA,motorB,touchB,motorC,touchC)
    armUp(motorB,touchB)
    goToA(motorC,touchC)
    goToBccw(motorC)
    resetGripper(motorA)
    pause(2)
end
function task5(motorA,motorB,motorC,touchB,sonicSensor)
    catchBall(motorA,motorB,touchB,sonicSensor)
    goToC(motorC)
    releaseBall(motorA,motorB,touchB,sonicSensor)
    goToBcw(motorC)
end
function task6(motorA,motorB,motorC,touchB,touchC,sonicSensor)
    goToC(motorC)
    catchBall(motorA,motorB,touchB,sonicSensor)
    goToA(motorC,touchC)
    releaseBall(motorA,motorB,touchB,sonicSensor)
    goToBccw(motorC)
end
function task7(motorA,motorB,motorC,touchB,touchC,sonicSensor)
    goToA(motorC,touchC)
    catchBall(motorA,motorB,touchB,sonicSensor)
    goToBccw(motorC)
    releaseBall(motorA,motorB,touchB,sonicSensor)
end
function task8(motorA,motorB,motorC,touchB,touchC,sonicSensor)
    catchBall(motorA,motorB,touchB,sonicSensor)
    goToA(motorC,touchC)
    releaseBall(motorA,motorB,touchB,sonicSensor)
    goToBccw(motorC)
end
function task9(motorA,motorB,motorC,touchB,touchC,sonicSensor)
    goToA(motorC,touchC)
    catchBall(motorA,motorB,touchB,sonicSensor)
    goToC(motorC)
    releaseBall(motorA,motorB,touchB,sonicSensor)
    goToBcw(motorC)
end
function task10(motorA,motorB,motorC,touchB,sonicSensor)
    goToC(motorC)
    catchBall(motorA,motorB,touchB,sonicSensor)
    goToBcw(motorC)
    releaseBall(motorA,motorB,touchB,sonicSensor)
end