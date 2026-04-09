#ifndef __INDEXER_H__ 
#define __INDEXER_H__ 

#include "../architecture/subsystem.h" 
#include "vex.h" 

class Indexer : public Subsystem {  

   private:   
    vex::motor indexerMotor;    
    vex::optical colorSensor; 
    double lastBlockTimestamp = 0;

   public:  
     using Subsystem::get;  
     
     static Indexer* globalRef;  
    
     Indexer(): 
     Subsystem( 
        "indexer", 
        { 
            (EntrySet){"is_on", EntryType::BOOL}, 
            (EntrySet){"detects_correct_color", EntryType::BOOL}, 
            (EntrySet){"detects_block", EntryType::BOOL}
        }
     ), 
     indexerMotor(vex::motor(vex::PORT7)), 
     colorSensor(vex::optical(vex::PORT9))
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
