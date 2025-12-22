#include "hooks.h" 

Hooks *Hooks::globalRef = nullptr; 

void Hooks::init(){ 
   set<bool>("is_on", true);     
   stop(); 
}  

void Hooks::periodic(){ 
   hookPiston1.set(RobotState::getStateOf("descore_out")); 
   hookPiston2.set(RobotState::getStateOf("descore_out"));
} 

void Hooks::updateTelemetry(){ 
   return;
} 

void Hooks::stop(){ 
   hookPiston1.set(false); 
   hookPiston2.set(false);
}