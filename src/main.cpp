#include "vex.h"
#include "architecture/robot.h"
#include <iostream>

#include "subsystems/drivebase.h" 
/*
#include "subsystems/indexer.h"
#include "subsystems/hood.h"
#include "subsystems/hopper.h"
#include "subsystems/intake.h"
#include "subsystems/matchloader.h"
#include "subsystems/hooks.h"  
*/

#include "commands.h"

using namespace vex;

competition Competition;
Robot robot;

void runTelemetry()
{
  robot.runTelemetryThread(false);
}

void freeDrive()
{
  thread telemetryThread = thread(runTelemetry);
  robot.driverControl();
}

void startCommandMatch(std::vector<CommandInterface *> commandGroup)
{
  robot.setAutonomousCommand(commandGroup);
  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(); });  
  while (!Competition.isEnabled()){ 
    this_thread::yield();
  }  
  robot.runTelemetryThread(false);
}

void driveCommandMatch(std::vector<CommandInterface *> commandGroup)
{
  robot.setAutonomousCommand(commandGroup);  // Registers the autonomous routine
  thread telemThread = thread(runTelemetry); // Start data logging
  robot.autonControl();                      // Runs the autonomous command
  robot.driverControl();                     // Free drive
}



int main()
{

  vexcodeInit();
  // Initialize subsystems
  Drivebase drive = Drivebase(2,1/3); // Tile location right 1 up 1

  robot.initialize();  
  
  RobotState::manuallyModifyState("is_team_color_blue", true);
  RobotState::manuallyModifyState("color_sensitive", false); 
  
  freeDrive();
 
}
