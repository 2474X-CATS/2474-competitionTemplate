#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "subsystems/drivebase.h"  
#include "subsystems/intake.h" 
#include "subsystems/indexer.h"
#include "subsystems/matchloader.h" 

#include "architecture/command.h" 
#include "helpers/trajectorycontroller.h"

typedef enum
{
  HIGH_GOAL = 1,
  MID_GOAL = 2,
  LOW_GOAL = 3
} Goal_Pos;

typedef enum
{
  NAT_ML_LEFT = 0,
  NAT_ML_RIGHT,
  NAT_HIGH_LEFT,
  NAT_HIGH_RIGHT,
  NAT_MID,
  NAT_LOW,
  NAT_PARK,
  FAR_ML_RIGHT,
  FAR_ML_LEFT,
  FAR_HIGH_LEFT,
  FAR_HIGH_RIGHT,
  FAR_MID,
  FAR_LOW,
  FAR_PARK
} Zones;

typedef enum
{
  EUCLIDEAN,
  MANHATTAN_XY,
  MANHATTAN_YX
} PathType;

class DrivePath : public Command<Drivebase, Intake>
{

protected:  

  Drivebase &drivebaseRef; 
  Intake &intakeRef; 

  vector<double> setpoints;
  bool turningFirst; 
  bool intaking;

  pidcontroller *turnPID = nullptr;
  TrapezoidalMotionProfile *drivingProfile = nullptr;

  bool initialized;

  bool isCounterClockwise;
  bool isGoingForward;
  double startingPoint[2];

  int operationsIndex;
  int numOfOperations;

  void initializeDrive();
  void initializeTurn();

  bool isDriveOver();
  bool isTurnOver();

  void drive();
  void turn();

  bool isTurning();
  bool isDriving();

  double getAngularError();

  void drivePeriodic();
  void turnPeriodic();

  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(vector<double> setpoints, bool turningFirst, bool intaking)
  {
    return new DrivePath(*Drivebase::globalRef, *Intake::globalRef, setpoints, turningFirst, intaking);
  }

  DrivePath(Drivebase &drive, Intake &intake, vector<double> setpoints, bool turningFirst, bool intaking) : Command<Drivebase, Intake>(drive, intake),
                                                                             drivebaseRef(drive), 
                                                                             intakeRef(intake),  
                                                                             intaking(intaking),
                                                                             setpoints(setpoints), 
                                                                             turningFirst(turningFirst), 
                                                                             initialized(false),
                                                                             operationsIndex(0),
                                                                             numOfOperations(setpoints.size() - 1) {};
};


class DriveToSetpoint : protected DrivePath
{
protected:
  double setpointX;
  double setpointY;
  double endingAngle;
  PathType pathType;

public:
  static CommandInterface *getCommand(double setpointX, double setpointY, double endingAngle, PathType pathType, bool intaking)
  {
    return new DriveToSetpoint(*Drivebase::globalRef, *Intake::globalRef, setpointX, setpointY, endingAngle, pathType, intaking);
  }

  DriveToSetpoint(Drivebase &drive, Intake &intake, double setpointX, double setpointY, double endingAngle, PathType pathType, bool intaking) : DrivePath(drive, intake, {}, true, intaking),
                                                                                                                 setpointX(setpointX),
                                                                                                                 setpointY(setpointY),
                                                                                                                 endingAngle(endingAngle),
                                                                                                                 pathType(pathType) {};

protected:
  void start() override;
}; 


class TurnToSetpoint : DriveToSetpoint
{

public: 

  static CommandInterface *getCommand(double setpointX, double setpointY)
  {
    return new TurnToSetpoint(*Drivebase::globalRef, *Intake::globalRef, setpointX, setpointY);
  }

  TurnToSetpoint(Drivebase &drive, Intake &intake, double setpointX, double setpointY) : DriveToSetpoint(drive, intake, setpointX, setpointY, -1, PathType::EUCLIDEAN, false){};
                                                                                                                                                                              
protected:
  void start() override;
};  

class CloseDistanceBetweenSetpoint : DriveToSetpoint
{

public: 
 
  static CommandInterface *getCommand(double setpointX, double setpointY, double distFrom, bool intaking)
  {
    return new CloseDistanceBetweenSetpoint(*Drivebase::globalRef, *Intake::globalRef,  setpointX, setpointY, distFrom, intaking);
  }

  CloseDistanceBetweenSetpoint(Drivebase &drive, Intake &intake, double setpointX, double setpointY, double distFrom, bool intaking) :  
                         DriveToSetpoint(drive, intake, setpointX, setpointY, -1, PathType::EUCLIDEAN, intaking), 
                         distFrom(distFrom) {};
                                                                                                                                                                              
protected:
  void start() override; 

private: 
  double distFrom; 
}; 


class DriveForwardForTime : Command<Drivebase, Intake>
{
private: 

  Drivebase &drivebaseRef;  
  Intake &intakeRef; 

  bool intaking;

  double percentage;
  double startingTime;
  double timeDuration; 


protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(double percentage, double timeDuration, bool intaking)
  {
    return new DriveForwardForTime(*Drivebase::globalRef, *Intake::globalRef,  percentage, timeDuration, intaking);
  }

  DriveForwardForTime(Drivebase &drivebase, Intake &intake, double percentage, double timeDuration, bool intaking) :  
                                                                                      Command<Drivebase, Intake>(drivebase, intake),  
                                                                                      intaking(intaking),
                                                                                      drivebaseRef(drivebase),  
                                                                                      intakeRef(intake),
                                                                                      percentage(percentage),
                                                                                      timeDuration(timeDuration)  
                                                                                      {};

  ~DriveForwardForTime() override = default;
}; 


class IntakeCubes : public Command<Intake>
{
private:
  Intake &intakeRef;

  double startingTime;
  double timeDuration;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(double timeDuration)
  {
    return new IntakeCubes(*Intake::globalRef, timeDuration);
  }

  IntakeCubes(Intake &intake, double timeDuration) : Command<Intake>(intake),
                                                                                      intakeRef(intake),
                                                                                      timeDuration(timeDuration) {};

  ~IntakeCubes() override = default;
};

class ScoreOnGoal : public Command<Intake, Indexer>
{
private:
  Intake &intakeRef;
  Indexer &indexerRef;

  double startingTime;
  double timeDuration;
  int goal;

public:
  static CommandInterface *getCommand(int goal, double timeDuration)
  {
    return new ScoreOnGoal(*Intake::globalRef, *Indexer::globalRef, goal, timeDuration);
  };

  ScoreOnGoal(Intake &intake, Indexer &indexer, int goal, double timeDuration) : Command<Intake, Indexer>(intake, indexer),
                                                                                                             intakeRef(intake),
                                                                                                             indexerRef(indexer),
                                                                                                             timeDuration(timeDuration),
                                                                                                             goal(goal) {};

  ~ScoreOnGoal() override = default;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
};


class DeployMatchloader : Command<Matchloader>
{
private:
  Matchloader &matchLoaderRef;
  bool isOut;
  bool ran = false;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(bool out)
  {
    return new DeployMatchloader(*Matchloader::globalRef, out);
  }

  DeployMatchloader(Matchloader &matchloader, bool out) : Command<Matchloader>(matchloader),
                                                          matchLoaderRef(matchloader),
                                                          isOut(out) {};

  ~DeployMatchloader() override = default;
};

class WaitFor : Command<DummySystem>
{
private:
  double timeDuration;
  double startTime;

public:
  static CommandInterface *getCommand(double timeDuration)
  {
    return new WaitFor(timeDuration);
  }

  WaitFor(double timeDuration) : Command<DummySystem>(GLOBAL_DUMMY),
                                 timeDuration(timeDuration) {};

  ~WaitFor() override = default;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
}; 

class ModifyRobotState : Command<DummySystem>
{ 
public:  

  string entryKey; 
  bool value; 

  static CommandInterface *getCommand(string key, bool val)
  {
    return new ModifyRobotState(key, val);
  }

  ModifyRobotState(string key, bool val) :  
                                 Command<DummySystem>(GLOBAL_DUMMY),
                                 entryKey(key), 
                                 value(val) {};

  ~ModifyRobotState() override = default;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
};


CommandInterface *DriveLinear(double distance, bool intaking);
CommandInterface *TurnToHeading(double angle);
CommandInterface *DriveToPoint(double setpointX, double setpointY, PathType pathType, bool intaking);
CommandInterface *AlignWithLocation(int locationIndex, double distance, PathType pathType, bool intaking);  
CommandInterface *GetWithinDistOfSetpoint(int locationIndex, double distFrom, bool intaking);
CommandInterface *FaceLocation(int locationIndex);  
CommandInterface *RamForward(double percentage, double duration, bool intaking);  

CommandInterface *CollectCubes(double timeDuration);   

CommandInterface *Score(Goal_Pos pos, double timeDuration); 

CommandInterface *EnableMatchloader(bool out);  


CommandInterface *Wait(double duration);   

CommandInterface *SetRobotState(string key, bool val); 
 






#endif