#include "SampleController.h"
#include <GCS_MAVLink/GCS.h>

void SampleController_init(void){
    gcs().send_text(MAV_SEVERITY_INFO, "Simulink Custom controller is initialized ");;
}
void SampleController_loop(void){
    Quaternion currAttQuat,desAttQuat;
    currAttQuat.initialise();
    desAttQuat.initialise();
    Vector3f currAngVel;
    // get current Attitude
    sl_ahrs_handle->getAttitudeQuat(currAttQuat);
    Vector3f currAtt;
    currAttQuat.to_euler(currAtt);
    // get current ang velocity
    currAngVel = sl_ahrs_handle->getAngularVelocity();
    //get Desired Attitude
    desAttQuat = sl_attControl_handle->getAttitudeSetpointQuat();
    Vector3f desAtt;
    desAttQuat.to_euler(desAtt);
    // Calculate attitude error
    Vector3f attError;
    for (int i = 0; i < 3; i++){
        attError[i] = desAtt[i]-currAtt[i];
    }
    // P controller
     Vector3f targetRate;
    targetRate[0] = 4 * attError[0];
    targetRate[1] = 4 * attError[1];
    targetRate[2] = 2 * attError[2];
    //PID controller
    Vector3f motorOut;
    motorOut[0] = 0.035*(targetRate[0]-currAngVel[0]);
    motorOut[1] = 0.035*(targetRate[1]-currAngVel[1]);
    motorOut[2] = 0.035*(targetRate[2]-currAngVel[2]);
    // set Torque value
    
    sl_motors_handle->setTorque(motorOut[0],motorOut[1],motorOut[2]);
    
    // get Current Altitude
    Vector3f currPOS;
    float currAlt;
    sl_ahrs_handle->getPositionNED(currPOS);
    //NED->NEU
    currAlt = -currPOS[2];
    currAlt = (currAlt<0)? 0:currAlt;
    // get Desired Altitude
    Vector3p desPos;
    float desAlt;
    desPos = sl_posControl_handle->getPositionSetPointNEU();
    //NEU and cm->m
    desAlt=static_cast<float>(desPos[2]*0.01);
    // get current velocity
    Vector3f currVel;
    sl_ahrs_handle->getVelocityNED(currVel);
    float velZ = -currVel[2];
    // altitude controller
    float targetAltRate = 1.5*(desAlt-currAlt);
    float tauthrust = 0.5*(targetAltRate - velZ);
    currAlt = tauthrust;
    // set thrust    
    sl_motors_handle->setThrust(tauthrust);
}