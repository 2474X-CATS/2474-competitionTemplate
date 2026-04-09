#ifndef __INTAKE_H__
#define __INTAKE_H__

#include "../architecture/subsystem.h"
#include "vex.h"

class Intake : public Subsystem
{

private:
  vex::motor intakeConveyor; 

public:
  using Subsystem::get;

  static Intake *globalRef;
  
  Intake() : Subsystem(
                 "intake",
                 { 
                  (EntrySet){"is_on", EntryType::BOOL},
                   }),
             intakeConveyor(vex::motor(vex::PORT8))
  {
    globalRef = this;
  }

  void init() override;
  void periodic() override;
  void updateTelemetry() override;
  void stop() override;

protected:
  using Subsystem::set;
};

#endif