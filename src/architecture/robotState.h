#ifndef __ROBOT_STATE_H__ 
#define __ROBOT_STATE_H__ 

#include "vex.h"
#include "subsystem.h"  

class RobotState : Subsystem {   
    
    protected: 
       using Subsystem::set;  
       
    private:  
       int mode = 1; // 1 is updating based on controller 2 is null state values 3 is manually set states (for auton)
       bool matchloaderHolding = false;    

       void updateStopped();  
       void updateRegular();  

      
    public:  
       using Subsystem::get; 
       using Subsystem::getFromInputs; 

       RobotState() :  
       Subsystem( 
        "robot-state", 
         { 
            (EntrySet){"intaking_to_hopper", EntryType::BOOL}, 
            (EntrySet){"scoring_high", EntryType::BOOL},
            (EntrySet){"scoring_mid", EntryType::BOOL}, 
            (EntrySet){"scoring_low", EntryType::BOOL}, 
            (EntrySet){"matchlaoder_out", EntryType::BOOL}
         }
       )
       {}; 
       
       void updateTelemetry() override;  
       void periodic() override; 
       void init() override;  
       void stop() override;    

       void setMode(); 
   
       void modifyStateManually(string key, bool val); 



}  

#endif 