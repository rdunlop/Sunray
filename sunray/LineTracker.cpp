// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "LineTracker.h"
#include "robot.h"
#include "StateEstimator.h"
#include "helper.h"
#include "pid.h"
#include "src/op/op.h"
#include "Stats.h"
#include "events.h"
#include "gps.h"


//PID pidLine(0.2, 0.01, 0); // not used
//PID pidAngle(2, 0.1, 0);  // not used
static Polygon circle(8);

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

LineTracker::LineTracker()
    : _est(&stateEstimator), _map(&maps), _mot(&motor), _op(&activeOp),
      _gpsSol([]{ return gps.solution; }) {}

LineTracker::LineTracker(StateEstimator& est, Map& mp, Motor& mot, Op*& op,
                         std::function<SolType()> gpsSol,
                         std::function<unsigned long()> millisFn)
    : _est(&est), _map(&mp), _mot(&mot), _op(&op),
      _gpsSol(std::move(gpsSol)), _millisFn(std::move(millisFn)) {}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void LineTracker::trackLine(bool runControl){
  // Time source: injected in tests so timers can be controlled; real millis() in production.
  auto ms = [this]() -> unsigned long {
    return _millisFn ? _millisFn() : (unsigned long)millis();
  };

  Point target = _map->targetPoint;
  Point lastTarget = _map->lastTargetPoint;
  float linear = 1.0;  
  bool mow = true;
  if (_est->stateOp == OP_DOCK) mow = false;
  float angular = 0;      
  float targetDelta = pointsAngle(_est->stateX, _est->stateY, target.x(), target.y());      
  if (_map->trackReverse) targetDelta = scalePI(targetDelta + PI);  
  targetDelta = scalePIangles(targetDelta, _est->stateDelta);
  trackerDiffDelta = distancePI(_est->stateDelta, targetDelta);                         
  _est->lateralError = distanceLineInfinite(_est->stateX, _est->stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float distToPath = distanceLine(_est->stateX, _est->stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        

  float lineDist = _map->distanceToTargetPoint(lastTarget.x(), lastTarget.y());
  /*if ((abs(lineDist-lastLineDist ) > 0.0) || (abs(distToPath) > 0.5)) {
    CONSOLE.print("distToPath=");
    CONSOLE.print(distToPath);
    CONSOLE.print(" x=");
    CONSOLE.print(_est->stateX);
    CONSOLE.print(" y=");    
    CONSOLE.print(_est->stateY);
    CONSOLE.print(" lastX=");    
    CONSOLE.print(lastTarget.x());
    CONSOLE.print(" lastY=");    
    CONSOLE.print(lastTarget.y());
    CONSOLE.print(" tgX=");    
    CONSOLE.print(target.x());
    CONSOLE.print(" tgY=");    
    CONSOLE.println(target.y());
    lastLineDist = lineDist;
  }*/
  float targetDist = _map->distanceToTargetPoint(_est->stateX, _est->stateY);
  
  float lastTargetDist = _map->distanceToLastTargetPoint(_est->stateX, _est->stateY);  
  if (SMOOTH_CURVES)
    targetReached = (targetDist < 0.2);    
  else 
    targetReached = (targetDist < TARGET_REACHED_TOLERANCE);

  
  // allow rotations only near last or next waypoint or if too far away from path
  // it might race between rotating mower and targetDist check below
  // if we race we still have rotateLeft or rotateRight true
  if ( (targetDist < 0.5) || (lastTargetDist < 0.5) || (fabs(distToPath) > 0.5) ||
       rotateLeft || rotateRight ) {
    if (SMOOTH_CURVES)
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 120);
    else     
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < TARGET_ANGLE_TOLERANCE);
  } else {
    // while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 45);       
    //angleToTargetFits = true;
  }

  //if (!angleToTargetFits) CONSOLE.println("!angleToTargetFits");

  if (!angleToTargetFits){
    // angular control (if angle to far away, rotate to next waypoint)
    linear = 0;
    angular = 29.0 / 180.0 * PI; //  29 degree/s (0.5 rad/s);               
     // decide for one rotation direction (and keep it)
    if ((!rotateLeft) && (!rotateRight)) {
      if (trackerDiffDelta < 0) rotateLeft = true;
        else rotateRight = true;      
    }
    if (rotateLeft) angular *= -1;
    if (fabs(trackerDiffDelta)/PI*180.0 < 90){
      rotateLeft = false;  // reset rotate direction
      rotateRight = false;
    }   
  } 
  else {
    // line control (stanley)    
    bool straight = _map->nextPointIsStraight();
    bool trackslow_allowed = true;

    rotateLeft = false;
    rotateRight = false;

    // in case of docking or undocking - check if trackslow is allowed
    if ( _map->isUndocking() || _map->isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        _map->getDockingPos(dockX, dockY, dockDelta);
        float dist_dock = distance(dockX, dockY, _est->stateX, _est->stateY);
        // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
        if (dist_dock > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
            trackslow_allowed = false;
        }
    }

    if (_map->trackSlow && trackslow_allowed) {
      // planner forces slow tracking (e.g. docking etc)
      linear = DOCK_LINEAR_SPEED; // 0.1           
    } else if (     ((_est->setSpeed > 0.2) && (_map->distanceToTargetPoint(_est->stateX, _est->stateY) < 0.5) && (!straight))   // approaching
          || ((_est->linearMotionStartTime != 0) && (ms() < _est->linearMotionStartTime + 3000))                      // leaving  
       ) 
    {
      linear = 0.1; // reduce speed when approaching/leaving waypoints          
      //CONSOLE.println("SLOW: approach")
    } 
    else {
      if ((_est->stateLocalizationMode == LOC_GPS) && (_gpsSol() == SOL_FLOAT)){        
        linear = min(_est->setSpeed, 0.1); // reduce speed for float solution
        //CONSOLE.println("SLOW: float");
      } else
        linear = _est->setSpeed;         // desired speed
      if (bumperDriver.nearObstacle()){
        linear = 0.1;  // slow down near obstacles 
        //CONSOLE.println("SLOW: BUMPER");      
      }
      if (lidarBumper.nearObstacle()){
        linear = 0.1;  // slow down near obstacles 
        //CONSOLE.println("SLOW: LiDAR");      
      }
      if (sonar.nearObstacle()) {
        linear = 0.1; // slow down near obstacles
        //CONSOLE.println("SLOW: sonar");      
      }
    }      
    // slow down speed in case of overload and overwrite all prior speed 
    if ( (_mot->motorLeftOverload) || (_mot->motorRightOverload) || (_mot->motorMowOverload) ){
      if (!printmotoroverload) {
          Logger.event(EVT_MOTOR_OVERLOAD_REDUCE_SPEED);
          CONSOLE.println("motor overload detected: reducing linear speed");
      }
      printmotoroverload = true;
      linear = min(linear, MOTOR_OVERLOAD_SPEED);  
      //CONSOLE.println("SLOW: overload");
    } else {
      printmotoroverload = false;
    }   
          
    //angula                                    r = 3.0 * trackerDiffDelta + 3.0 * lateralError;       // correct for path errors 
    float k = stanleyTrackingNormalK; // STANLEY_CONTROL_K_NORMAL;
    float p = stanleyTrackingNormalP; // STANLEY_CONTROL_P_NORMAL;    
    if (_map->trackSlow && trackslow_allowed) {
      k = stanleyTrackingSlowK; //STANLEY_CONTROL_K_SLOW;   
      p = stanleyTrackingSlowP; //STANLEY_CONTROL_P_SLOW;          
    }
    angular =  p * trackerDiffDelta + atan2(k * _est->lateralError, (0.001 + fabs(_mot->linearSpeedSet)));       // correct for path errors           
    /*pidLine.w = 0;              
    pidLine.x = _est->lateralError;
    pidLine.max_output = PI;
    pidLine.y_min = -PI;
    pidLine.y_max = PI;
    pidLine.compute();
    angular = -pidLine.y;   */
    //CONSOLE.print(_est->lateralError);        
    //CONSOLE.print(",");        
    //CONSOLE.println(angular/PI*180.0);            
    if (_map->trackReverse) linear *= -1;   // reverse line tracking needs negative speed
    // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
    //if (!SMOOTH_CURVES) angular = max(-PI/16, min(PI/16, angular)); 
  }
  // check some pre-conditions that can make linear+angular speed zero
  if ((_est->stateLocalizationMode == LOC_GPS) && (_est->fixTimeout != 0)){
    if (ms() > _est->lastFixTime + _est->fixTimeout * 1000.0){
      (*_op)->onGpsFixTimeout();        
    }           
  }     

  if (_est->stateLocalizationMode == LOC_GPS){
    if  ((_gpsSol() == SOL_FIXED) || (_gpsSol() == SOL_FLOAT)){        
      if (abs(linear) > 0.06) {
        if ((ms() > _est->linearMotionStartTime + 5000) && (_est->stateGroundSpeed < 0.03)){
          // if in linear motion and not enough ground speed => obstacle
          //if ( (GPS_SPEED_DETECTION) && (!_map->isUndocking()) ) { 
          if (GPS_SPEED_DETECTION) {         
            CONSOLE.println("gps no speed => obstacle!");
            stats.statMowGPSNoSpeedCounter++;
            Logger.event(EVT_NO_GPS_SPEED_OBSTACLE);
            triggerObstacle();
            return;
          }
        }
      }  
    } else {
      // no gps solution
      if (REQUIRE_VALID_GPS){
        CONSOLE.println("WARN: no gps solution!");
        (*_op)->onGpsNoSignal();
      }
    }
  }

  // tree-canopy detection: track how long GPS has been non-FIXED
  if (_est->stateLocalizationMode == LOC_GPS) {
    if (_gpsSol() != SOL_FIXED) {
      if (gpsDegradedSince == 0) gpsDegradedSince = ms();
    } else {
      gpsDegradedSince = 0;
    }
  } else {
    gpsDegradedSince = 0;
  }

  if (_est->stateLocalizationMode == LOC_APRIL_TAG){
    if (!_est->stateAprilTagFound){
      linear = 0; // wait until april-tag found 
      angular = 0; 
    } else {
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      //linear = 0; // wait until april-tag found 
      //angular = 0; 
    }
  }
  if (_est->stateLocalizationMode == LOC_REFLECTOR_TAG){
    if (!_est->stateReflectorTagFound){
      linear = 0; // wait until reflector-tag found 
      angular = 0; 
    } else {
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      float maxAngular = 0.015;  // 0.02
      float maxLinear = 0.05;      
      angular =  max(min(1.0 * trackerDiffDelta, maxAngular), -maxAngular);
      angular =  max(min(angular, maxAngular), -maxAngular);      
      linear = 0.05;      
      if (_map->trackReverse) linear = -0.05;   // reverse line tracking needs negative speed           
    }
  }
  if (_est->stateLocalizationMode == LOC_GUIDANCE_SHEET){
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      angular = 0;
  }

  // gps-jump/false fix check
  if (KIDNAP_DETECT){
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;     
    if ( _map->isUndocking() || _map->isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        _map->getDockingPos(dockX, dockY, dockDelta);
        float dist = distance(dockX, dockY, _est->stateX, _est->stateY);
        // check if current distance to docking station is below
        // KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK to trigger KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK
        if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK) {
            allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
        }
    }    
    if ((_est->stateLocalizationMode == LOC_GPS) && (fabs(distToPath) > allowedPathTolerance)){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped){
        stateKidnapped = true;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(distToPath);
        (*_op)->onKidnapped(stateKidnapped);
      }            
    } else {
      if (stateKidnapped) {
        stateKidnapped = false;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(distToPath);
        (*_op)->onKidnapped(stateKidnapped);        
      }
    }
  }
   
  // in any case, turn off mower motor if lifted 
  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
  if (detectLift()) mow = false;
  
  if (mow)  { 
    if (ms() < _mot->motorMowSpinUpTime + 10000){
       // wait until mowing motor is running
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      linear = 0;
      angular = 0;          
    }
  }

  if (runControl){
    if (angleToTargetFits != langleToTargetFits) {
        //CONSOLE.print("angleToTargetFits: ");
        //CONSOLE.print(angleToTargetFits);
        //CONSOLE.print(" trackerDiffDelta: ");
        //CONSOLE.println(trackerDiffDelta);
        langleToTargetFits = angleToTargetFits;
    }

    #ifdef DOCK_REFLECTOR_TAG
      if (_est->stateLocalizationMode == LOC_REFLECTOR_TAG){             
        CONSOLE.print("loc=");
        if (_est->stateLocalizationMode == LOC_APRIL_TAG) CONSOLE.print("april");
        if (_est->stateLocalizationMode == LOC_GPS) CONSOLE.print("gps");
        if (_est->stateLocalizationMode == LOC_GUIDANCE_SHEET) CONSOLE.print("guide");    
        if (_est->stateLocalizationMode == LOC_REFLECTOR_TAG) CONSOLE.print("reflector");        
        CONSOLE.print(" tagFound=");
        CONSOLE.print(_est->stateReflectorTagFound);
        CONSOLE.print(" tagOut=");
        CONSOLE.print(_est->stateReflectorTagOutsideFound);      
        CONSOLE.print(" reflX=");
        CONSOLE.print(_est->stateXReflectorTag);
        CONSOLE.print(" reflY=");
        CONSOLE.print(_est->stateYReflectorTag);
        CONSOLE.print(" mow=");
        CONSOLE.print(mow);      
        CONSOLE.print(" shouldDock=");
        CONSOLE.print(_map->shouldDock);      
        CONSOLE.print(" trackRev=");
        CONSOLE.print(_map->trackReverse);
        CONSOLE.print(" lin=");
        CONSOLE.print(linear);
        CONSOLE.print(" ang=");
        CONSOLE.print(angular);    
        CONSOLE.print(" isBetwLNTLDockPt=");
        CONSOLE.print(_map->isBetweenLastAndNextToLastDockPoint());
        CONSOLE.print(" dockPtIdx=");
        CONSOLE.print(_map->dockPointsIdx);
        CONSOLE.print(" freePtIdx=");
        CONSOLE.print(_map->freePointsIdx);
        CONSOLE.print(" wayMode=");
        if (_map->wayMode == WAY_DOCK) CONSOLE.print("WAY_DOCK");
        if (_map->wayMode == WAY_MOW) CONSOLE.print("WAY_MOW");
        if (_map->wayMode == WAY_FREE) CONSOLE.print("WAY_FREE");
        CONSOLE.println();
      }
    #endif

    _mot->setLinearAngularSpeed(linear, angular);      
    _mot->setMowState(mow);    
  }

  //if (!_map->isTargetingLastDockPoint()){
  bool treeSkip = GPS_TREE_SKIP
               && (_map->wayMode == WAY_MOW)
               && (gpsDegradedSince != 0)
               && (ms() - gpsDegradedSince > GPS_TREE_SKIP_TIMEOUT)
               && (targetDist < GPS_TREE_SKIP_MAX_DIST);
  if (_est->stateLocalizationMode != LOC_REFLECTOR_TAG){
    if (targetReached || treeSkip){
      if (treeSkip && !targetReached){
        CONSOLE.print("WARN: GPS degraded under tree (");
        CONSOLE.print((ms() - gpsDegradedSince) / 1000.0, 1);
        CONSOLE.print("s, dist=");
        CONSOLE.print(targetDist, 2);
        CONSOLE.println("m) - forcing waypoint advance");
      }
      gpsDegradedSince = 0;  // reset timer so next waypoint gets a clean window
      rotateLeft = false;
      rotateRight = false;
      (*_op)->onTargetReached();
      bool straight = _map->nextPointIsStraight();
      if (!_map->nextPoint(false,_est->stateX,_est->stateY)){
        // finish
        (*_op)->onNoFurtherWaypoints();
      } else {
        // next waypoint
        //if (!straight) angleToTargetFits = false;
      }
    }
  }  
}
