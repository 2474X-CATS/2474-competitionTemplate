#ifndef __TRAP_MOTION_H__ 
#define __TRAP_MOTION_H__ 

typedef struct {  
   double position; 
   double velocity; 
   double acceleration; 
} TrapezoidalSetpoint;  

typedef struct { 
  double maxVelocity; 
  double maxAcceleration; 
} TrapezoidConstants;

class TrapezoidalMotionProfile {  
   
    private: 
      double accelDist; 
      double accelTime; 

      double cruiseDist; 
      double cruiseTime; 

      double decelDist; 
      double decelTime; 
      
      double maxVelocity; 
      double maxAcceleration; 
 
      double setpoint; 

      double startingTimestamp;  

      double toleranceVel = 0.0; 
      double tolerancePos = 0.0; 

      double calculateVelocity(double time); 
      double calculateAcceleration(double time);  
      
      TrapezoidConstants consts;

      void init(double startingVelocity, double finalVelocity); 

    public: 

      TrapezoidalMotionProfile(TrapezoidConstants constants, double setpoint, double startingVelocity, double finalVelocity): 
      maxVelocity(constants.maxVelocity), 
      maxAcceleration(constants.maxAcceleration), 
      setpoint(setpoint), 
      consts(constants)
      { 
        init(startingVelocity, finalVelocity);
      };    

      TrapezoidalMotionProfile(TrapezoidConstants constants, double setpoint) : TrapezoidalMotionProfile(constants, setpoint, 0, 0) {}; 
      
      TrapezoidalMotionProfile(TrapezoidConstants constants, double setpoint, double startingVelocity) : TrapezoidalMotionProfile(constants, setpoint, startingVelocity, 0) {}; 

      TrapezoidalSetpoint generateSetpoint(double time);

      void setPositionTolerance(double posTol); 
      void setVelocityTolerance(double velTol);   

      double getTotalDuration(); 
      double getStartTime();    

      double getAccelDist();  
      double getDecelDist();
      double getCruiseDist();

      double calculatePosition(double time);  
      double convertPosToTime(double position);

      bool atGoal(double currentPosition, double currentVelocity);  

      double getMaxVelocity(); 
      double getMaxAcceleration(); 

      void setLastTimestamp(double timestamp);

};



#endif