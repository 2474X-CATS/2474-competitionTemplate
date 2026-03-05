#ifndef __PATH_H__ 
#define __PATH_H__ 

#include "bezier.h" 
#include "trapezoidalMotion.h"

typedef struct { 
   //array<double,2> referencePoint; 
   double referenceHeading; //What angle we need to be facing
   double angleGradient; //How much our angle setpoint has changed
   double frameVelocity; //The calculated base speed derived from a trapzeoidal motion profile 
} PathFrameOutput;

class Path {   

   private:     

      BezierCurve* curve = nullptr;  
      TrapezoidalMotionProfile* curveProfile = nullptr; 
    
      double lDist;  
      double pathProgress = 0;   
      double lastAngleSetpoint = -1; //Obscure number not likely to be replicated

      
   public:  
      Path(BezierCurve* curve, TrapezoidConstants constants, double lookAheadDistance){};   

      

};




#endif 