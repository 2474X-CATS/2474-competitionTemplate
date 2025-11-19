#include "subsystem.h"
#include "vex.h" 

vector<Subsystem *> Subsystem::systems;

Subsystem::Subsystem(string tableLabel, vector<EntrySet> entryNames) : label(tableLabel)
{
   Subsystem::systems.push_back(this);
   Telemetry::inst.registerSubtable(this->label, entryNames);
};

void Subsystem::updateSystems()
{
   for (Subsystem *system : systems)
   {
      system->periodic();
   }
};

void Subsystem::initSystems()
{
   for (Subsystem *system : systems)
   {
      system->init();
   }
};

void Subsystem::refreshTelemetry()
{
   for (Subsystem *system : systems)
   {
      system->updateTelemetry();
   }
}; 

void Subsystem::stopAll(){ 
  for (Subsystem *system : systems)
   {
      system->stop();
   }
};   

Subsystem* Subsystem::getSubsystem(int index){ 
  return systems.at(index);
} 

//----------------------------------------------------------------------

RobotState* RobotState::globalState = nullptr; 

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
   set<bool>("toggling_hood", Controller.ButtonL1.pressing()); 
}

void RobotState::updateStopped(){ 
   set<bool>("intake_to_hopper", false);  
   set<bool>("scoring_high", false);  
   set<bool>("scoring_mid", false);
   set<bool>("scoring_low", false);  
   set<bool>("matchloader_out", false); 
   set<bool>("toggling_hood", false);
}; 

void RobotState::modifyStateManually(string key, bool val){ 
   globalState->set<bool>(key, val); 
};

bool RobotState::getStateOf(string key){ 
   return globalState->get<bool>(key); 
}; 

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


DummySystem GLOBAL_DUMMY;

