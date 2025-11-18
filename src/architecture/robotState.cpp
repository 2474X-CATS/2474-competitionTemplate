#include "robotState.h" 

void RobotState::updateTelemetry(){  

  switch (mode){ 
   case 1: 
      updateRegular(); 
      break 
   case 2: 
      updateStopped();   
      break;
   case 3: 
   default: 
      break;
  } 
   
};  

void RobotState::updateRegular(){ 
   set<bool>("intake_to_hopper", Controller.ButtonY.pressing());  
   set<bool>("scoring_high", Controller.ButtonR1.pressing());  
   set<bool>("scoring_mid", Controller.ButtonR2.pressing());
   set<bool>("scoring_low", Controller.ButtonB.pressing());  
   set<bool>("matchloader_out", Controller.ButtonL2.pressing());
}

void RobotState::updateStopped(){ 
   set<bool>("intake_to_hopper", false);  
   set<bool>("scoring_high", false);  
   set<bool>("scoring_mid", false);
   set<bool>("scoring_low", false);  
   set<bool>("matchloader_out", false);
}


void RobotState::init(){ 
   return;
}; 

void RobotState::periodic(){ 
   return;
}; 

void RobotState::stop(){ 
   return;
} 

void RobotState::setMode(int mode){ 
   this.mode = mode;  
   if (mode == 3){ 
     updateStopped();
   }
}; 

void RobotState::modifyStateManually(string key, bool val){ 
   set<bool>("robot-state", val);  
}; 
