#ifndef __PATH_H__ 
#define __PATH_H__ 

#include "../utilities/bezier.h" 
#include "trapezoidalMotion.h"  
#include "pidcontroller.h"
#include "math.h"

typedef struct { 
   double linearVelocity; //The calculated base speed derived from a trapzeoidal motion profile  
   double angularVelocity; //Based on the calculated linear velocity and the angular error calculate the angular velocity 
} PathFrameOutput; 

typedef struct{ 
   array<double, 2> point; 
   double progressT; //Closest t point
} BezierReferencePoint;

class HomingPath {   

   private:     
      static double TUNED_LDIST; 
      static double TUNED_L_SCALE;  
      static double OPTIMUM_TOLERANCE;

      BezierCurve* curve = nullptr;   
      pidcontroller* turnController = nullptr; 
      TrapezoidalMotionProfile* curveProfile = nullptr;   

      double lDist;     
      double maxCentripetalAcceleration;  
      double lastVelocity = 0;  

      double lastTimestamp; 
     
      double pathProgress = 0;
      double deriveMaxVelocity(double radius);   

      double distanceTolerance = 0;  
      double distanceTraveled = 0; 

      double k_scale;

      array<double, 2> endpoint; 
      
      void updateAnchorPoint(double closestT); 
 
      double getOptimumLookaheadDist(double velocity); 
 
      TrapezoidalMotionProfile* getProfile(); 
      
      BezierReferencePoint findReferencePoint(double x, double y, double lookAhead); 

      double getAcceleration();  

      double getPathProgress(); 

      BezierCurve* getCurve();


   public:  
      HomingPath(BezierCurve* curve, TrapezoidConstants motionConstants, PIDConstants pidconstants, double lookAheadDistance, double k_scale, double maxCentripAccel, double distTolerance);    
      
      HomingPath(array<array<double,2>, 3> points, TrapezoidConstants motionConstants, PIDConstants pidconstants, double maxCentripAccel);

      PathFrameOutput calculateFrameOutput(double x, double y, double heading, double timestamp); 

      void init(double timestamp);  

      bool completed(double x, double y);   

}; 

class CirclePath {   

   private:     
   
      pidcontroller* turnController = nullptr; 
      TrapezoidalMotionProfile* profile = nullptr;   

      double radius; 
      double lastTimestamp = -1;    
      double lastOmega = 0;

      double arcLength;

      bool straight = false; 
      int turningDirection = 0; 
      int drivingDirection = 1;
      
      array<double, 2> startPoint; 
      //array<double, 2> centroid;
      array<double, 2> endpoint;  

      double projectedHeading;  
      
      double getAngularVelocity(double linearVelocity, double heading, double timestamp);   


   public:  
      
      CirclePath(array<double, 2> startingPoint, double heading, array<double, 2> endPoint, bool cuttingCorners, double maxCentripetalAcceleration, TrapezoidConstants consts, PIDConstants pidconsts);

      PathFrameOutput calculateFrameOutput(double x, double y, double heading, double timestamp); 

      void init(double stamp);  

      bool completed(double timestamp);   

};




#endif 