#include "commands.h"

////////////////////////////////////////////////////////////


void DrivePath::start()
{
    return;
}

void DrivePath::periodic()
{
    if (isDriving())
    {
        drivePeriodic();
    }
    else
    {
        turnPeriodic();
    } 
    intakeRef.periodic(); 
}

bool DrivePath::isOver()
{
    return operationsIndex > numOfOperations;
}

void DrivePath::end()
{
   RobotState::manuallyModifyState("intaking", false);
}

void DrivePath::drivePeriodic()
{
    if (!initialized)
    {
        initializeDrive();
        initialized = true;  
        RobotState::manuallyModifyState("intaking", intaking);
    }
    else
    {
        if (isDriveOver())
        {
            operationsIndex += 1;
            initialized = false;
            drivebaseRef.stop();
            delete drivingProfile; 
            RobotState::manuallyModifyState("intaking", false);
        }
        else
        {
            drive();
        }
    }
}

void DrivePath::turnPeriodic()
{
    if (!initialized)
    {
        initializeTurn();
        initialized = true;
    }
    else
    {
        if (isTurnOver())
        {
            operationsIndex += 1;
            initialized = false;
            drivebaseRef.stop();
            delete turnPID;
        }
        else
        {
            turn();
        }
    }
}

bool DrivePath::isDriving()
{
    return turningFirst ? operationsIndex % 2 == 1 : operationsIndex % 2 == 0;
}

bool DrivePath::isTurning()
{
    return turningFirst ? operationsIndex % 2 == 0 : operationsIndex % 2 == 1;
}

bool DrivePath::isDriveOver()
{
    return (Brain.Timer.time(vex::sec) - drivingProfile->getStartTime()) >= drivingProfile->getTotalDuration(); 
}

void DrivePath::initializeDrive()
{
    isGoingForward = setpoints.at(operationsIndex) > 0;
    drivingProfile = new TrapezoidalMotionProfile(drivebaseRef.getMotionConstants(), abs(setpoints.at(operationsIndex)), Brain.Timer.time(vex::sec));
    
}

void DrivePath::drive()
{  
    double output; 
    TrapezoidalSetpoint setpoint = drivingProfile->generateSetpoint(Brain.Timer.time(vex::sec));  
    output = setpoint.velocity;
    if (!isGoingForward)
        output *= -1;
    drivebaseRef.manualDriveForward(output); 
}


bool DrivePath::isTurnOver()
{
    return turnPID->atSetpoint(getAngularError());
}

void DrivePath::initializeTurn()
{
    double startAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");

    double angleSetpoint = setpoints.at(operationsIndex);

    double normalDist = startAngle > angleSetpoint ? (360 - startAngle) + angleSetpoint : angleSetpoint - startAngle;
    isCounterClockwise = true;

    if (normalDist > 180)
    {
        isCounterClockwise = false;
        normalDist = 360 - normalDist;
    }

    turnPID = new pidcontroller(drivebaseRef.getTurningPID(), 0);
    turnPID->setLastTimestamp(Brain.Timer.time()); 

}

void DrivePath::turn()
{
    double output = turnPID->calculate(getAngularError(), Brain.Timer.time());
    if (isCounterClockwise)
    {
        output *= -1;
    }
    drivebaseRef.manualTurnClockwise(-output);
}

double DrivePath::getAngularError()
{
    double currentAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");
    double angleSetpoint = setpoints.at(operationsIndex);

    double dist = currentAngle > angleSetpoint ? (360 - currentAngle) + angleSetpoint : angleSetpoint - currentAngle;

    if (!isCounterClockwise)
    {
        dist = (360 - dist);
    }

    return dist;
}

void DriveToSetpoint::start()
{

    double startingX = drivebaseRef.get<double>("Pos_X");
    double startingY = drivebaseRef.get<double>("Pos_Y");

    double overallDist = hypot(startingX - setpointX, startingY - setpointY); 
    
    double overallAngle = fmod((atan2(startingY - setpointY, startingX - setpointX) / M_PI * 180 + 180), 360);

    double xDist = setpointX - startingX;
    double yDist = setpointY - startingY;

    switch (pathType)
    {
    case EUCLIDEAN:
        setpoints.push_back(overallAngle);
        setpoints.push_back(overallDist);
        break;
    case MANHATTAN_XY:
        setpoints.push_back((xDist >= 0 ? 0 : 180));
        setpoints.push_back(fabs(xDist));
        setpoints.push_back((yDist >= 0 ? 90 : 270));
        setpoints.push_back(fabs(yDist));
        break;
    case MANHATTAN_YX:
        setpoints.push_back((yDist >= 0 ? 90 : 270));
        setpoints.push_back(fabs(yDist));
        setpoints.push_back((xDist >= 0 ? 0 : 180));
        setpoints.push_back(fabs(xDist));
        break;
    }

    if (endingAngle != -1 && pathType != PathType::EUCLIDEAN)
        setpoints.push_back(endingAngle);

    numOfOperations = setpoints.size() - 1;
}

//////////////////////////////////////////////////////////// 

void TurnToSetpoint::start(){ 
   DriveToSetpoint::start(); 
   setpoints.pop_back(); 
   numOfOperations--;
};  

void CloseDistanceBetweenSetpoint::start(){ 
   DriveToSetpoint::start(); 
   operationsIndex++; 
   setpoints[1] = setpoints[1] - distFrom;
}; 

////////////////////////////////////////////////////////////

void DriveForwardForTime::start()
{ 
    RobotState::manuallyModifyState("intaking", intaking);
    startingTime = Brain.Timer.time(vex::msec);   
};

void DriveForwardForTime::periodic()
{
    drivebaseRef.arcadeDrive(percentage * 100, 0);  
    intakeRef.periodic();
};

bool DriveForwardForTime::isOver()
{
    return Brain.Timer.time(vex::msec) - startingTime >= timeDuration;
}

void DriveForwardForTime::end()
{  
    RobotState::manuallyModifyState("intaking", false);
    drivebaseRef.stop();
} 

//////////////////////////////////////////////////////////////////////////////// 

void IntakeCubes::start()
{ 
    RobotState::manuallyModifyState("intaking", true); 
    startingTime = Brain.Timer.time(vex::msec);
};

void IntakeCubes::periodic()
{
    intakeRef.periodic();
};

bool IntakeCubes::isOver()
{
    return Brain.Timer.time(vex::msec) - startingTime >= timeDuration;
}

void IntakeCubes::end()
{
    RobotState::manuallyModifyState("intaking", false);
}

////////////////////////////////////////////////////////////

void ScoreOnGoal::start()
{
    // Top: 1
    // Mid: 2
    // Low: 3
    switch (goal)
    {
    case 1:
        RobotState::manuallyModifyState("scoring_high", true);
        break;
    case 2:
        RobotState::manuallyModifyState("scoring_mid", true);
        break;
    case 3:
        RobotState::manuallyModifyState("scoring_low", true);
        break;
    default:
        break;
    }
    startingTime = Brain.Timer.time();
};

void ScoreOnGoal::periodic()
{
    indexerRef.periodic();
    intakeRef.periodic();
}

bool ScoreOnGoal::isOver()
{
    return Brain.Timer.time() - startingTime >= timeDuration;
}

void ScoreOnGoal::end()
{
    switch (goal)
    {
    case 1:
        RobotState::manuallyModifyState("scoring_high", false);
        break;
    case 2:
        RobotState::manuallyModifyState("scoring_mid", false);
        break;
    case 3:
        RobotState::manuallyModifyState("scoring_low", false);
        break;
    default:
        break;
    }
}

////////////////////////////////////////////////////////////

void DeployMatchloader::start()
{
    RobotState::manuallyModifyState("matchloader_out", isOut);
}

void DeployMatchloader::periodic()
{
    matchLoaderRef.periodic();
    ran = true;
}

bool DeployMatchloader::isOver()
{
    return ran;
}

void DeployMatchloader::end()
{
    return;
}

////////////////////////////////////////////////////////////


void WaitFor::start()
{
    startTime = Brain.Timer.time();
}

void WaitFor::periodic()
{
    return;
}

bool WaitFor::isOver()
{
    return Brain.Timer.time() - startTime >= timeDuration;
}

void WaitFor::end()
{
    return;
} 

void ModifyRobotState::start()
{
   RobotState::manuallyModifyState(entryKey, value);
}

void ModifyRobotState::periodic()
{
    return;
}

bool ModifyRobotState::isOver()
{
    return true;
}

void ModifyRobotState::end()
{
    return;
}

////////////////////////////////////////////////////////////

CommandInterface *DriveLinear(double distance, bool intaking)
{
    return DrivePath::getCommand({distance}, false, intaking);
};

CommandInterface *TurnToHeading(double angle)
{
    return DrivePath::getCommand({angle}, true, false);
};

CommandInterface *DriveToPoint(double setpointX, double setpointY, PathType pathType, bool intaking)
{ 
    return DriveToSetpoint::getCommand(setpointX, setpointY, -1, pathType, intaking);
};

CommandInterface *AlignWithLocation(int locationIndex, double distance, PathType pathType, bool intaking)
{
    array<double, 2> setpoint = Drivebase::getLocation(locationIndex)->getProjectedSetpoint(distance);
    return DriveToSetpoint::getCommand(setpoint[0], setpoint[1], Drivebase::getLocation(locationIndex)->getPerfectEntranceAngle(), pathType, intaking);
};

CommandInterface *FaceLocation(int locationIndex)
{
    array<double, 2> setpoint = Drivebase::getLocation(locationIndex)->getProjectedSetpoint(0);
    return TurnToSetpoint::getCommand(setpoint[0], setpoint[1]);
};

CommandInterface* RamForward(double percentage, double duration, bool intaking){ 
    return DriveForwardForTime::getCommand(percentage, duration, intaking);
};   

CommandInterface* GetWithinDistOfSetpoint(int locationIndex, double distFrom, bool intaking){  
    array<double, 2> setpoint = Drivebase::getLocation(locationIndex)->getProjectedSetpoint(0);
    return CloseDistanceBetweenSetpoint::getCommand(setpoint[0], setpoint[1], distFrom, intaking);
} 


CommandInterface *CollectCubes(double timeDuration){ 
    return IntakeCubes::getCommand(timeDuration); 
};   

CommandInterface *Score(Goal_Pos pos, double timeDuration){ 
    return ScoreOnGoal::getCommand(pos, timeDuration);  
}; 

CommandInterface *EnableMatchloader(bool out){ 
    return DeployMatchloader::getCommand(out);
};  

CommandInterface *SetRobotState(string key, bool val){ 
    return ModifyRobotState::getCommand(key, val);
}; 

CommandInterface *Wait(double duration)
{
    return WaitFor::getCommand(duration);
};  



