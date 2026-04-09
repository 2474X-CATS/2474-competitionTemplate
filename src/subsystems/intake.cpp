#include "intake.h"

Intake *Intake::globalRef = nullptr;

void Intake::init()
{
   intakeConveyor.setBrake(vex::brakeType::brake);
   set<bool>("is_on", true);
   stop();
}

void Intake::periodic()
{ 
   if (RobotState::getStateOf("intaking"))
   {
      intakeConveyor.setVelocity(100, vex::velocityUnits::pct); // Inwards
      intakeConveyor.spin(vex::directionType::fwd);
   }
   else if (RobotState::getStateOf("scoring_high") || RobotState::getStateOf("scoring_mid"))
   {
      intakeConveyor.setVelocity(100, vex::velocityUnits::pct); // Inwards
      intakeConveyor.spin(vex::directionType::fwd);
   }
   else if (RobotState::getStateOf("scoring_low") || RobotState::getStateOf("outtaking"))
   {
      intakeConveyor.setVelocity(-100, vex::velocityUnits::pct); // Outwards
      intakeConveyor.spin(vex::directionType::fwd);
   }
   else
   {
      stop();
   }
}

void Intake::updateTelemetry()
{
  return;
}

void Intake::stop()
{
   intakeConveyor.stop();
}
