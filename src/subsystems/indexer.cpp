#include "indexer.h" 

Indexer *Indexer::globalRef = nullptr;  


void Indexer::init(){   

   indexerMotor.setBrake(vex::brakeType::brake);   

   colorSensor.setLightPower(100); 
   colorSensor.setLight(vex::ledState::on);   

   colorSensor.integrationTime(20);
   
   set<bool>("is_on", true);     
   stop();  

}  

void Indexer::periodic(){    
   bool ableToScore = true; 
   
   if (RobotState::getStateOf("in_autonomous")){ 
     if (RobotState::getStateOf("color_sensitive")){  
      ableToScore = get<bool>("detects_correct_color");
     }
   }   
   if (ableToScore && RobotState::getStateOf("scoring_high")){  

     indexerMotor.setVelocity(-100, vex::velocityUnits::rpm); 
     indexerMotor.spin(vex::directionType::fwd);   

   } else if (RobotState::getStateOf("scoring_mid")){     

     indexerMotor.setVelocity(100, vex::velocityUnits::rpm);
     indexerMotor.spin(vex::directionType::fwd); 

   } else { 
     stop();
   }   

} 

void Indexer::updateTelemetry(){  

   vex::color intolerableColor; 
   if (RobotState::getStateOf("is_team_color_blue")){ 
      intolerableColor = vex::red;
   } else { 
      intolerableColor = vex::blue;
   } 
  
   if (colorSensor.isNearObject()){ 
    lastBlockTimestamp = Brain.Timer.time(vex::msec);
   } 

   set<bool>("detects_block", Brain.Timer.time(vex::msec) - lastBlockTimestamp < 200); // 200 ms tolerance
   set<bool>("detects_correct_color", colorSensor.color() != intolerableColor); 
   
}  



void Indexer::stop(){  
   indexerMotor.setVelocity(0, vex::percentUnits::pct); 
   indexerMotor.spin(vex::directionType::fwd);
}