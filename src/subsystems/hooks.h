#ifndef __HOOKS_H__ 
#define __HOOKS_H__ 

#include "../architecture/subsystem.h" 
#include "vex.h" 

class Hooks : public Subsystem {  

   private:   
    vex::pneumatics hookPiston1;  
    vex::pneumatics hookPiston2;  

   public:  
     using Subsystem::get;  
     
     static Hooks* globalRef; 

     Hooks(): 
     Subsystem( 
        "hooks", 
        { 
            (EntrySet){"is_on", EntryType::BOOL}
        }
     ), 
     hookPiston1(vex::pneumatics(Brain.ThreeWirePort.D)), 
     hookPiston2(vex::pneumatics(Brain.ThreeWirePort.B))
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