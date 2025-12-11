#ifndef __INDEXER_H__
#define __INDEXER_H__

#include "../architecture/subsystem.h"
#include "vex.h"

typedef enum { 
   HIGH, 
   MID,  
   STORAGE,
   NONE
} Feed; 

class Indexer : public Subsystem
{
public:
   using Subsystem::get;
   
   static Indexer* globalRef; 

   Indexer() : Subsystem(
                   "indexer",
                   { 
                     (EntrySet){"detects_blocks_high", EntryType::BOOL},  
                     (EntrySet){"detects_blocks_mid", EntryType::BOOL},
                     (EntrySet){"is_block_blue", EntryType::BOOL}, 
                     (EntrySet){"detects_jam", EntryType::BOOL}
                   } 
                  ), 
                  indexerMotor(vex::motor(vex::PORT19)), 
                  colorSensor(vex::optical(vex::PORT6)), 
                  distanceSensor(vex::distance(vex::PORT13))
                  { 
                     globalRef = this;
                   } 
               
   void init() override;
   void periodic() override;
   void updateTelemetry() override; 
   void stop() override; 

   
protected:
   using Subsystem::set;   

private:   

   vex::motor indexerMotor;  
   vex::optical colorSensor;  
   vex::distance distanceSensor;

   static double ABSOLUTE_INDEXER_SPEED;  

   static array<int, 3> BLUE_RGB; 
   static array<int, 3> RED_RGB;   

   static double DISTANCE_THRESHOLD;

   void spinAtPercent(double percentage);
   
   bool hasBlueBlock();

   double distBlue(vex::optical::rgbc rgb); 
   double distRed(vex::optical::rgbc rgb);

   double getExpectedVelocity(); 

   
};

#endif