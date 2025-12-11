#ifndef __HOPPER_H__
#define __HOPPER_H__

#include "../architecture/subsystem.h"


class Hopper : public Subsystem
{
public:
    using Subsystem ::get;

    static Hopper *globalRef;

    Hopper() : Subsystem(
                   "hopper",
                   {  
                    (EntrySet){"detects_jam", EntryType::BOOL}, 
                    (EntrySet){"primed", EntryType::BOOL}
                   }),
               hopperMotor(vex::motor(vex::PORT18, false)), 
               distanceSensor(vex::distance(vex::PORT5))
    {
        globalRef = this;
    }

    void init() override;
    void periodic() override;
    void updateTelemetry() override;
    void stop() override;

private: 

    vex::motor hopperMotor;
    vex::distance distanceSensor; 

    static double ABSOLUTE_HOPPER_SPEED;
    static double DISTANCE_THRESHOLD;

    void spinAtPercent(double percentage); 

    double getExpectedVelocity(); 
    
};

#endif
