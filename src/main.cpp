#include "vex.h"
#include "architecture/robot.h"
#include <iostream>

#include "commands.h"
#include "gui/autonInit.h"

using namespace vex;

competition Competition;
Robot robot;

typedef struct
{
  string name;
  string desc;
  array<vector<CommandInterface *>, 2> autos;
} Routine;

AutonOption convertRoutineToOption(Routine routine, int index)
{
  AutonOption option;
  option.description = routine.desc;
  option.name = routine.name;
  option.hasLeftSide = !routine.autos.at(0).empty();
  option.hasRightSide = !routine.autos.at(1).empty();
  option.index = index;
  return option;
}

vector<AutonOption> getOptionVector(vector<Routine> routines)
{
  vector<AutonOption> res;
  for (int index = 0; index < routines.size(); index++)
  {
    res.push_back(convertRoutineToOption(routines.at(index), index));
  }
  return res;
}

void runTelemetry()
{
  robot.runTelemetryThread();
}

void awaitStartingSignal()
{
  LoadingScreen();
  Sprite::frameLoop();
}

int scheduleCallbacks()
{

  // Add a pause where you press a button to start
  //awaitStartingSignal(); 

  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(); });

  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  return 0;
}

void testDrive()
{
  thread telemetryThread = thread(runTelemetry);
  robot.driverControl();
}

void testAuto(vector<CommandInterface *> auton, bool startingLeft)
{ 
  if (startingLeft){ 
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425); 
  } else { 
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
  }
  robot.setAutonomousCommand(auton);
  thread telemThread = thread(runTelemetry);
  robot.autonControl();
  robot.driverControl();
}

void configurateAutonomous(vector<Routine> routines)
{
  ColorPicker colorChooser = ColorPicker(135 + 115, 200);
  SidePicker sidePicker = SidePicker(235 + 115, 200);
  RoutineCatalog catalog = RoutineCatalog(10, 10, getOptionVector(routines));
  AutonDisplay(250, 80, &catalog, &sidePicker);
  ExitBlock(250, 10);
  Sprite::frameLoop();

  int sideIndex;
  int autonIndex = catalog.getCurrentAuto().index;

  if (sidePicker.getIsLeft())
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425);
    sideIndex = 0;
  }
  else
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
    sideIndex = 1;
  }

  robot.setAutonomousCommand(routines.at(autonIndex).autos.at(sideIndex));
  RobotState::manuallyModifyState("is_team_color_blue", colorChooser.getIsBlue());
}

void startCommandSkillsMatch(vector<CommandInterface *> commandGroup, bool startingLeft)
{  
  RobotState::manuallyModifyState("in_skills", true);
  if (startingLeft)
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425);
  }
  else
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
  }
  robot.setAutonomousCommand(commandGroup);
  thread callBackTrigger = thread(scheduleCallbacks);
  robot.runTelemetryThread();
}

void startCommandCompetitiveMatch(vector<Routine> routines)
{
  configurateAutonomous(routines);
  thread callBackTrigger = thread(scheduleCallbacks);
  robot.runTelemetryThread();
}

void driveCommandMatch(vector<Routine> routines)
{
  configurateAutonomous(routines);
  thread telemThread = thread(runTelemetry);
  awaitStartingSignal();
  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  robot.autonControl();
  robot.driverControl();
}

/*
-------------------------------------------------------------------------------------------------------------------------------------------------
     _              _______   _____    _____
    / \    |     |     |     |     |   |
   /___\   |     |     |     |     |   -----
  /     \  |_____|     |     |_____|   ____|
 --------------------------------------------------------------------------------------------------------------------------------------------------
*/


vector<CommandInterface *> closed_side_left()
{
  return {
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.925), // Get cubes near low
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.925, ROBOT_LENGTH_MM/2+50, true),  
      DriveForwardForTime::getCommand(0.125, 1000, true),   
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),  
      
      ModifyRobotState::getCommand("is_drive_inverted", true),
      
      //TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.3), 

      DrivePath::getCommand({50, TILE_SIZE_MM*0.95}, true, true),  
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000),    
       
      ModifyRobotState::getCommand("is_drive_inverted", false), 
      DriveForwardForTime::getCommand(0.2, 300, false),  
      TurnToLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM/2 + 250),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM+80),
      DrivePath::getCommand({270}, true, false),
      
      DriveForwardForTime::getCommand(0.25, 600, true), 
      IntakeCubes::getCommand(500),    
      ModifyRobotState::getCommand("is_drive_inverted", true),   
      
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 0, -125, true),  
      
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1250), 
      DisengageHighGoal::getCommand(0.2, 750), 

      
    };
}

vector<CommandInterface*> closed_side_right(){ 
  return {
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2+25, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400),
     
      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 2000), 
      DeployMatchloader::getCommand(true),   
     
  
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5),
      
      TurnToLocation(Zones::NAT_ML_RIGHT),
      CloseDistanceBetween(Zones::NAT_ML_RIGHT, ROBOT_LENGTH_MM/2+250, 0, true),
      DriveForwardForTime::getCommand(0.2, 400, true), 
      IntakeCubes::getCommand(625),   
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      TurnToLocation(Zones::NAT_HIGH_RIGHT),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, ROBOT_LENGTH_MM / 2, 0, true),  
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.2, 750),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1250), 
      DisengageHighGoal::getCommand(0.2, 500), 

      DrivePath::getCommand({30}, true, false),
      DeployDescore::getCommand(true),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM/2) + 100)), 
      DrivePath::getCommand({90}, true, false), 
      DriveForwardForTime::getCommand(0.3, 750, false),  
      WaitFor::getCommand(5000)
    };
}

vector<CommandInterface*> switcheroo_left(){ 
  return {  
     DeployMatchloader::getCommand(true), 
     DriveForwardForTime::getCommand(0.2, 300, false),   
     
     TurnToLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM), 
     SlantedAlignWithX::getCommand(TILE_SIZE_MM + 30),

     DrivePath::getCommand({270}, true, false),  
     WaitFor::getCommand(300),
     DriveForwardForTime::getCommand(0.3, 750, true),
     IntakeCubes::getCommand(500),  

     ModifyRobotState::getCommand("is_drive_inverted", true),  
     WaitFor::getCommand(50),
     CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 0, -75, true), 
     Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.35, 100), 
     ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000), 
     DeployMatchloader::getCommand(false), 

     ModifyRobotState::getCommand("is_drive_inverted", false),  
     WaitFor::getCommand(50),   
     FlatAlignWithY::getCommand(TILE_SIZE_MM * 1),  

     TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM*1.3), 
     CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM*1.3, ROBOT_LENGTH_MM/2+25, false), 
     DriveForwardForTime::getCommand(0.125, 1000, true),   
     DeployMatchloader::getCommand(true),
     IntakeCubes::getCommand(600),  
     
     ModifyRobotState::getCommand("is_drive_inverted", true), 
     TurnToLocation(Zones::NAT_MID), 
     DrivePath::getCommand({TILE_SIZE_MM * 1.175}, false, false), 
     ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1500) 
     
  };
}

vector<CommandInterface*> solo_awp_left(){ 
  return { 
    
  };
} 

vector<CommandInterface*> solo_awp_right(){ 
  return { 
    DriveForwardForTime::getCommand(0.2, 300, false),  
    DriveToLocation(Zones::NAT_ML_RIGHT, TILE_SIZE_MM, PathType::EUCLIDEAN, false), 
    TurnToLocation(Zones::NAT_ML_RIGHT),  

    /* 
      Matchload
    */ 

    ModifyRobotState::getCommand("is_drive_inverted", true), 
    CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, ROBOT_LENGTH_MM/2, 100, true), 
    Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.2, 400), 
    ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),  

    ModifyRobotState::getCommand("is_drive_inverted", false), 
    DriveForwardForTime::getCommand(0.2, 500, false), 
    TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM*0.95), 
    CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2+100, false), 
    DeployMatchloader::getCommand(true), 
    DrivePath::getCommand({ROBOT_LENGTH_MM/2+100}, false, true), 
    IntakeCubes::getCommand(400),
    DeployMatchloader::getCommand(false), 

    TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.95), 
    CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2+100, true), 
    DeployMatchloader::getCommand(true), 
    DrivePath::getCommand({ROBOT_LENGTH_MM/2+100}, false, true), 
    IntakeCubes::getCommand(400),
    DeployMatchloader::getCommand(false), 

    ModifyRobotState::getCommand("is_drive_inverted", true), 
    TurnToLocation(Zones::NAT_MID), 
    CloseDistanceBetween(Zones::NAT_MID, ROBOT_LENGTH_MM/2, 100, false), 
    ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1500), 

    ModifyRobotState::getCommand("is_drive_inverted", false), 
    SlantedAlignWithX::getCommand(TILE_SIZE_MM), 
    TurnToLocation(Zones::NAT_ML_LEFT), 

    /* 
      Matchload 
    */ 

    ModifyRobotState::getCommand("is_drive_inverted", true), 
    CloseDistanceBetween(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM/2, 100, true), 
    Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.2, 400), 
    ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500)
  };
}

vector<CommandInterface *> renegade_right()
{
  return {    
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.885),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.885, ROBOT_LENGTH_MM/2+50, true),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),  
         
      ModifyRobotState::getCommand("is_drive_inverted", false),   
      TurnToLocation(Zones::NAT_ML_RIGHT, TILE_SIZE_MM*1.125), 
      SlantedAlignWithX::getCommand(TILE_SIZE_MM*5+35), 
      DrivePath::getCommand({270}, true, false),
       
      DriveForwardForTime::getCommand(0.25, 800, true),
      IntakeCubes::getCommand(500), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, -75, 0, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.3, 100), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),   
      DisengageHighGoal::getCommand(0.225, 700), 
 
      DrivePath::getCommand({30}, true, false),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM/2) + 120)), 
      DrivePath::getCommand({90}, true, false),  
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.4, 825, false),  
      WaitFor::getCommand(5000)
      
    };
} 

vector<CommandInterface *> renegade_left()
{
  return {   
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.925), // Get cubes near low
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.925, ROBOT_LENGTH_MM/2+50, true),  
      DriveForwardForTime::getCommand(0.125, 1000, true),   
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),   
         
      ModifyRobotState::getCommand("is_drive_inverted", false), 
      TurnToLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM), 
      SlantedAlignWithX::getCommand(TILE_SIZE_MM+80), 
      DrivePath::getCommand({270}, true, false),
       
      DriveForwardForTime::getCommand(0.25, 800, true),
      IntakeCubes::getCommand(500), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, -75, 0, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.3, 100), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),   
      DisengageHighGoal::getCommand(0.225, 700), 
 
      DrivePath::getCommand({30}, true, false),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM + ((ROBOT_WIDTH_MM/2) + 100)), 
      DrivePath::getCommand({90}, true, false),  
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.4, 825, false),  
      WaitFor::getCommand(5000)
      
    };
}

vector<CommandInterface*> control_rush_left(){ 
  return {  
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.925),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.925, ROBOT_LENGTH_MM/2+50, true),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),    

      DeployMatchloader::getCommand(false), 
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.4),
      ModifyRobotState::getCommand("is_drive_inverted", true),  
      WaitFor::getCommand(50),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM),   
      TurnToLocation(Zones::NAT_HIGH_LEFT),   

      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 0, -75, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.2, 100), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),  
      DisengageHighGoal::getCommand(0.225, 700),

      DrivePath::getCommand({30}, true, false),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM + ((ROBOT_WIDTH_MM/2) + 100)), 
      DrivePath::getCommand({90}, true, false),  
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.4, 850, false),  
      WaitFor::getCommand(5000) 
      

  };
}

vector<CommandInterface*> control_rush_right(){ 
  return {  
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.875),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.875, ROBOT_LENGTH_MM/2+50, true),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400), 

      DeployMatchloader::getCommand(false), 
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.125),
      ModifyRobotState::getCommand("is_drive_inverted", true),  
      WaitFor::getCommand(50),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM*5),   
      TurnToLocation(Zones::NAT_HIGH_RIGHT),  

      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, 0, -75, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.2, 100), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),  
      DisengageHighGoal::getCommand(0.225, 700),

      DrivePath::getCommand({30}, true, false),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM/2) + 100)), 
      DrivePath::getCommand({90}, true, false),  
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.4, 825, false),  
      WaitFor::getCommand(5000)
  };
}

vector<CommandInterface*>  center_cleanup_left(){ 
  return { 
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400),
      ModifyRobotState::getCommand("is_drive_inverted", true), 

      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000), 
      ModifyRobotState::getCommand("is_drive_inverted", false),  

      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM), 
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM, ROBOT_LENGTH_MM/2, false),  
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2+50}, false, true),  
      IntakeCubes::getCommand(400), 
      DeployMatchloader::getCommand(false), 
      
      TurnToLocation(Zones::NAT_LOW), 
      CloseDistanceBetween(Zones::NAT_LOW, ROBOT_LENGTH_MM/2, 0, false), 
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1500),
  };
} 

vector<CommandInterface*> center_cleanup_right(){ 
  return { 
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      IntakeCubes::getCommand(400),
      DeployMatchloader::getCommand(false),

      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1500), 
  

      DrivePath::getCommand({-TILE_SIZE_MM*0.8}, false, false),  
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM), 
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM, ROBOT_LENGTH_MM/2, false),  
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2+50}, false, true),  
      IntakeCubes::getCommand(400), 
      DeployMatchloader::getCommand(false), 
      ModifyRobotState::getCommand("is_drive_inverted", true), 

      TurnToLocation(Zones::NAT_MID), 
      CloseDistanceBetween(Zones::NAT_MID, ROBOT_LENGTH_MM/2, 0, false), 
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000),
  };
}

vector<CommandInterface *> auto_skills()
{
  return {       
     
      DeployDescore::getCommand(false),
      DriveForwardForTime::getCommand(0.2, 450, false),
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      DriveToLocation(Zones::NAT_HIGH_RIGHT, TILE_SIZE_MM * 0.5, PathType::EUCLIDEAN, false),  
      TurnToLocation(Zones::NAT_HIGH_RIGHT), 
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, -12.5, 0, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.45, 400), 

      DeployMatchloader::getCommand(true), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 500),  

      ModifyRobotState::getCommand("is_drive_inverted", false), 
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_ML_RIGHT, 300, 0, false),   
      DriveForwardForTime::getCommand(0.25, 250, true),   
      IntakeCubes::getCommand(2000),
      

      //////////////////////////////////////////
      
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      DriveForwardForTime::getCommand(0.25, 500, true),  
  
      DriveToSetpoint::getCommand(TILE_SIZE_MM * 4.25, TILE_SIZE_MM * 1.6, -1, PathType::EUCLIDEAN, true),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 4.55), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5),   
      
      //DrivePath::getCommand({270}, true, false),   
      TurnToLocation(Zones::FAR_HIGH_RIGHT),
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0.45, 1550),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000),    
      DisengageHighGoal::getCommand(0.1, 250),
      DeployMatchloader::getCommand(true), 

  
      ModifyRobotState::getCommand("is_drive_inverted", false), 
      WaitFor::getCommand(50), 
      CloseDistanceBetween(Zones::FAR_ML_RIGHT, 300, 0, false), 
      DriveForwardForTime::getCommand(0.25, 250, true),   
      
      IntakeCubes::getCommand(2000),   
    

      ModifyRobotState::getCommand("is_drive_inverted", true),  
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, ROBOT_LENGTH_MM/2, 0, true), 
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0.45, 800),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000),    
      
      
      DisengageHighGoal::getCommand(0.25, 750),  
      
      ModifyRobotState::getCommand("is_drive_inverted", true),  
       
      DrivePath::getCommand({180, TILE_SIZE_MM * 4+15}, true, false),
      TurnToLocation(Zones::FAR_HIGH_LEFT),
      CloseDistanceBetween(Zones::FAR_HIGH_LEFT, ROBOT_LENGTH_MM/2, 0, false),  
      
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_LEFT, 0.45, 950), 
      WaitFor::getCommand(50),

      ModifyRobotState::getCommand("is_drive_inverted", false), 
      WaitFor::getCommand(50), 
      CloseDistanceBetween(Zones::FAR_ML_LEFT, 300, 0, false),
      DriveForwardForTime::getCommand(0.25, 250, true),   
      IntakeCubes::getCommand(2250),   

      
      ModifyRobotState::getCommand("is_drive_inverted", true),  
      WaitFor::getCommand(50), 

      DriveForwardForTime::getCommand(0.3, 500, false),   
      DriveToSetpoint::getCommand(TILE_SIZE_MM*1.75, TILE_SIZE_MM*4, 270, PathType::EUCLIDEAN, true), 
      FlatAlignWithY::getCommand(TILE_SIZE_MM*1.5), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM),  

      TurnToLocation(Zones::NAT_HIGH_LEFT), 

      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.4, 1300),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000),   
      DisengageHighGoal::getCommand(0.2, 350),

      ModifyRobotState::getCommand("is_drive_inverted", false),  
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_ML_LEFT, 300, 0, false), 
      DriveForwardForTime::getCommand(0.25, 250, true),   
      IntakeCubes::getCommand(2200),    


      ModifyRobotState::getCommand("is_drive_inverted", true),  
      WaitFor::getCommand(50),
      //TurnToLocation(Zones::NAT_HIGH_LEFT),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 150, 0, true),  
      
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.45, 500),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2250),   

    
      DeployMatchloader::getCommand(false),  
      DisengageHighGoal::getCommand(0.1, 250),

      ModifyRobotState::getCommand("is_drive_inverted", false),  
      WaitFor::getCommand(50),
      SlantedAlignWithY::getCommand(TILE_SIZE_MM), 
      TurnToSetpoint::getCommand(TILE_SIZE_MM * 2.65, TILE_SIZE_MM * 0.1), 
      DrivePath::getCommand({TILE_SIZE_MM * 2.35},false, true), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 750)  
  };
}

vector<CommandInterface*> empty(){ 
  return {};
} 

vector<CommandInterface*> do_nothing(){ 
  return {  
    IntakeCubes::getCommand(500)
  };
}

vector<CommandInterface*> drive_forward(){ 
  return { 
     DriveForwardForTime::getCommand(0.2, 500, false)
  };
}

//------------------------------>-------------------------------------------------------------------------------------------------------------------

int main()
{

  vexcodeInit();

  //--------------------DONT MODIFY (MOSTLY)-----------------

  Drivebase drive = Drivebase(0,0); 
  Intake intake;
  Indexer indexer;
  Matchloader matchloader;
  Hooks hooks;

  robot.initialize();

  RobotState::manuallyModifyState("color_sensitive", false); // <- We don't have color-sort currently
  RobotState::manuallyModifyState("is_counterclockwise", false);

  //-------------------ROUTINE CREATION-------------------
  
  vector<Routine> routines;

  routines.push_back(
      (Routine){
          "Default [WHEN IN DOUBT]",
          "If on the left side: >>[4 blocks in mid - matchload 3 cubes - 3 blocks in long]. If right: [matchload 3 cubes - 7 blocks in long]",
          {
           closed_side_left(),
           renegade_right()
          } 
      } 
  );  

  routines.push_back( 
    (Routine){ 
      "7 Block High", 
      "Symmetrical: Gets middle cubes and matchloaded cubes and scores on the high goal", 
      { 
        renegade_left(), 
        renegade_right(), 
      }
    }
  );  

  routines.push_back( 
     (Routine){ 
        "2/3 AWP", 
        "Symmetrical: Grab near cubes, score on the center goal: (mid : left, low right), then score matchloaded cubes on high", 
        { 
          closed_side_left(), 
          closed_side_right()
        }
     }
  ); 

  routines.push_back( 
    (Routine){ 
      "Control Rush", 
      "Symmetrical: Score 3 blocks on the high goal, push to control then camp. [USE STRATEGICALLY]", 
      { 
        control_rush_left(), 
        control_rush_right(), 
      }
    }
  ); 
   
  routines.push_back(
     (Routine){ 
        "Switcheroo", 
        "Left ONLY: Switch the order of scoring for 7 block auto [high goal -> mid goal]", 
        { 
          switcheroo_left(), 
          empty()
        }
     }
  ); 

  routines.push_back( 
    (Routine){ 
        "Center Goal Clean", 
        "Symmetrical: Score a combined 7 blocks on center goals - order depends on side [left mid, right low]", 
        { 
          center_cleanup_left(), 
          center_cleanup_right()
        }
    }
  );


  routines.push_back(  
    (Routine){ 
      "Getting Carried", 
      "Our alliance has a FOR SURE AWP: (left : nothing), (right : forward a smidge)", 
      { 
        do_nothing(), 
        drive_forward()
      } 
    }
  );
  
  
  //-----------------------PROTOCOL------------------------


 //startCommandCompetitiveMatch(routines);  //Uncomment when loading up for a comp   
 //startCommandSkillsMatch(auto_skills(), false);  //Uncomment when loading up for skills 
 //testDrive();//Uncomment when getting driver practice 

 
 testAuto( 
   renegade_right(),
   false
 );     
 
 
 
 
 
 
 
 
    
 


}
