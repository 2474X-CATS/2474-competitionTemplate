#include "path.h"
#include "vex.h" 

CirclePath::CirclePath(array<double, 2> startingPoint, double heading, array<double, 2> endPoint, bool cuttingCorners, double maxCentripetalAcceleration, TrapezoidConstants consts, PIDConstants pidconsts){  
  
  this->startPoint = startingPoint; 
  this->endpoint = endPoint;  
  this->projectedHeading = heading; 
  
  double dist = hypot(startingPoint[0] - endPoint[0], startingPoint[1] - endPoint[1]); 
  
  double targetHeading = fmod( (atan2(endPoint[1] - startingPoint[1], endPoint[0] - startingPoint[0])) / M_PI * 180 + 360, 360);
  
  double angleDiff = targetHeading - heading;   
  
  if (fabs(angleDiff) > 180){ 
     angleDiff = (360 - fabs(angleDiff)) * -1 * copysign(1, angleDiff);
  }       

  if (fabs(angleDiff) > 90 && cuttingCorners){
     angleDiff = fmod((180 - fabs(angleDiff)) * -1 * copysign(1, angleDiff), 180); 
     drivingDirection = -1;
  }
  
  if (fabs(angleDiff) < 1e-6){  

    straight = true;  
    this->radius = 1e7; 
    this->arcLength = dist;  

  } else {  
    angleDiff = angleDiff / 180 * M_PI;   

    this->turningDirection = static_cast<int>(copysign(1, angleDiff)); 

    this->radius = dist / angleDiff;//(2 * sin(angleDiff/2)); 
    this->arcLength = fabs(this->radius * angleDiff);    
    this->radius /= 2; 
  }   


  consts.maxVelocity = std::min<double>( 
      consts.maxVelocity, 
      sqrt(maxCentripetalAcceleration * this->radius)
  ); 

  this->turnController = new pidcontroller(pidconsts, 0); 
  this->profile = new TrapezoidalMotionProfile(consts, this->arcLength);  
  
};

bool CirclePath::completed(double timestamp){ 
  return (timestamp - profile->getStartTime()) > profile->getTotalDuration();
}

double CirclePath::getAngularVelocity(double linearVelocity, double heading, double timestamp){  
    
    if (straight){  
      return 0;
    }  
  
    this->projectedHeading += this->lastOmega * ((timestamp - lastTimestamp) / 1000); 

    double omega = fabs((linearVelocity / this->radius) / M_PI * 180) * turningDirection; //What the angular velocity should be 
    this->lastOmega = omega; 

    double error = this->projectedHeading - heading; //Now we can update the projected heading
    
    if (fabs(error) > 180){ 
     error = (360 - fabs(error)) * -1 * copysign(1, error);
    }   

    omega += turnController->calculate(error, timestamp); //Corrects for what the heading should be

    return omega;
}  

PathFrameOutput CirclePath::calculateFrameOutput(double x, double y, double heading, double timestamp){ 
    PathFrameOutput output;  
    output.linearVelocity = profile->generateSetpoint(timestamp).velocity * drivingDirection;  
    output.angularVelocity = getAngularVelocity(output.linearVelocity, heading, timestamp);   
    this->lastTimestamp = timestamp; 
    return output;
} 

void CirclePath::init(double timestamp){ 
   this->lastTimestamp = timestamp; 
   this->profile->setLastTimestamp(timestamp);   
   this->turnController->setLastTimestamp(timestamp); 
}