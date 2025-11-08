#include "intake.h"
#include "vex.h"


Intake* Intake::globalRef = nullptr; 

double Intake::ABSOLUTE_INTAKE_SPEED = 200;  

void Intake::init()
{  
    Telemetry::inst.placeValueAt<double>(intakeMotor.temperature(), "Motor_Temps","IntakeMotor");
    set<bool>("blockedByCubes", false); 
};

void Intake::periodic()
{
    if (shouldIntake())
    { 
        intake();
    }
    else if (shouldOuttake())
    {
        outtake();
    }
    else
    {
        stop();
    }
    
};

void Intake::updateTelemetry()
{ 
   return;
}

bool Intake::shouldIntake()
{
    return getFromInputs<bool>("Controller/Button_Y") ||
           getFromInputs<bool>("Controller/Button_R1") ||
           getFromInputs<bool>("Controller/Button_R2");
}

bool Intake::shouldOuttake()
{
    return getFromInputs<bool>("Controller/Button_B") ||  
           getFromInputs<bool>("Controller/Button_RIGHT");
} 

void Intake::intake(){  
    intakeMotor.setVelocity(-ABSOLUTE_INTAKE_SPEED, vex::velocityUnits::rpm); 
    intakeMotor.spin(vex::directionType::fwd);
}  

void Intake::outtake(){ 
    intakeMotor.setVelocity(ABSOLUTE_INTAKE_SPEED * 0.5, vex::velocityUnits::rpm); 
    intakeMotor.spin(vex::directionType::fwd);
}  

void Intake::stop(){ 
    intakeMotor.setVelocity(0, vex::percentUnits::pct); 
    intakeMotor.spin(vex::directionType::fwd);
} 


