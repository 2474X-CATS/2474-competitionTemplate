#include "path.h" 



PathFrameOutput Path::calculateFrameOutput(double x, double y, double heading){ 
   PathFrameOutput res;  
   BezierReferencePoint refPoint = findReferencePoint(x,y);
   
   /*
   double dist = sqrt(pow(x - refPoint[0],2) + pow(y - refPoint[1], 2)); 
   double desiredHeading = (acos((refPoint[0] - x) / dist) + M_PI) / (2 * M_PI) * 360; 
   
   double angleDiff = desiredHeading - heading;  
   if (fabs(angleDiff) > 180){ 
      angleDiff =(-1 * copysign(1, angleDiff)) * (360 - fabs(angleDiff));
   }  

   TrapezoidalSetpoint velocitySetpoint = curveProfile->generateSetpoint(curveProfile->convertPosToTime(curve->getPathLength() * refPoint.progressT));
   
   double linearVelocity = velocitySetpoint.velocity; 
   double angularVelocity = angleDiff;
   */ 

   return res;
} 

BezierReferencePoint Path::findReferencePoint(double x, double y){ 
    BezierReferencePoint res;  
    return res;
}