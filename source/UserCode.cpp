#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]

const float dt = 1.0f / 500.0f;  //[s] period between successive calls to MainLoop

// ∗B e f o r e ∗MainLoop
Vec3f estGyroBias = Vec3f (0,0,0);

//intialize values for estimated roll/pitch/yaw
float estRoll = 0;//0.5236;
float estPitch = 0;//0.5236;
float estYaw = 0;//0.5236;
float rollM = 0;
float pitchM = 0;
float rho = 0.0001; //define rho

float l = 33e-3f;
float K = 0.01f;
float l2 = 1.0f/(4.0f*l);
float k2 = 1.0f/(4.0f*K);
float M[4][4] = {{0.25,l2,-l2,k2}, {0.25,-l2,-l2,-k2} , {0.25,-l2,l2,k2}, {0.25,l2,l2,-k2}};

float cp[4] = {0,0,0,0};


Vec3f cmdAngAcc = Vec3f(0,0,0);
Vec3f cmdAngVel = Vec3f(0,0,0);
Vec3f desired_angle = Vec3f(0,0,0);

// intialize values for vertical state estimator
float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;
float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;

// time constsfor the attitude control
const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

float des1 = 1.0f;
float des2 = 0.0f;

float pos1 = 0.0f;
float pos2 = 0.0f;

float err1 = 0.0f;
float err2 = 0.0f;

float desHeight  = 1.5f;


MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here!
  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.userInput.buttonBlue" is true if the
  // blue button is pushed, false otherwise.

  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;

  //integrated positions
     pos1 = pos1 + estVelocity_1*dt;
     pos2 = pos2 + estVelocity_2*dt;
     err1 = -(des1 - pos1)/(12.0f);
     err2 = -(des2 - pos2)/(11.0f);

  if (in.currentTime < 1.0f) {
      estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
    }
  Vec3f rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias; //subtract bias from Gyros

  rollM = (in.imuMeasurement.accelerometer.y)/gravity; //phi measurement
  pitchM = -(in.imuMeasurement.accelerometer.x)/gravity; //theta measurement
  //estPitch = estPitch + dt*rateGyro_corr.y;
  float ep = estPitch + dt*rateGyro_corr.y;
  estPitch = (1-rho)*ep + rho*pitchM; //calculate the estimated pitch using the accelerometer measurements
  //estRoll = estRoll + dt*rateGyro_corr.x;
  float er = estRoll + dt*rateGyro_corr.x;
  estRoll = (1-rho)*er + rho*rollM; //calculate the estimated roll using the accelerometer measurements
  estYaw = estYaw + dt*rateGyro_corr.z; //calculate the estimated yaw

  //following lines add the estimated data to csv file
  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;

  // height estimator, prediction step
  estHeight = estHeight + estVelocity_3*dt; // prediction step for height
  estVelocity_3 = estVelocity_3 + 0*dt; //assume const, prediction step for v_3


  // height estimator correction step
  float const mixHeight = 0.3f; // mixing const for height estimator
  if (in.heightSensor.updated){ // checks that new measurement available
    if(in.heightSensor.value < 5.0f){  // check that measurement is reasonable
      float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch); // compute measured height
      estHeight = (1 -mixHeight)*estHeight + mixHeight*hMeas; // correct height estimate
      float v3Meas = (hMeas - lastHeightMeas_meas)/(in.currentTime-lastHeightMeas_time); // compute v_3 estimate
      estVelocity_3 = (1 - mixHeight) * estVelocity_3 + mixHeight * v3Meas; // correct v_3 estimate
      lastHeightMeas_meas = hMeas; // store this height estimate for next range value
      lastHeightMeas_time = in.currentTime; // store this height estimate for next range value
    }
  }

  // horizaontal estimator prediction step
  estVelocity_1 = estVelocity_1 - 0*dt; //+ err1;
  estVelocity_2 = estVelocity_2 + 0*dt; //+ err2;

  // horizontal estimator correction step
  float const mixHorizVel = 0.2f;//0.1f;
  if (in.opticalFlowSensor.updated){
    float sigma_1 = in.opticalFlowSensor.value_x;
    float sigma_2 = in.opticalFlowSensor.value_y;

    float div = (cosf(estRoll)*cosf(estPitch));
    if(div>0.5f){
      float deltaPredict = estHeight / div; //this is delta in the eqn

      float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
      float v2Meas = (-sigma_2 + in.imuMeasurement.rateGyro.x) * deltaPredict;

      estVelocity_1 = (1 - mixHorizVel) * estVelocity_1 + mixHorizVel*v1Meas;
      estVelocity_2 = (1 - mixHorizVel) * estVelocity_2 + mixHorizVel*v2Meas;
    }

  }



  //define time constants
  float const timeConstant_rollRate = 0.025f;//0.04f; //[s]
  float const timeConstant_pitchRate = timeConstant_rollRate;
  float const timeConstant_yawRate = 0.1f; //[s]
  float const timeConstant_rollAngle = 0.12f;//0.12f; //[s]
  float const timeConstant_pitchAngle = timeConstant_rollAngle;
  float const timeConstant_yawAngle = 0.2f; //[s]

  //time const for horiz vel

  const float timeConst_horizVel = 10.0f;//2.0f;

  // desired accel components control code
  float desAcc1 = -(1/ timeConst_horizVel) * (estVelocity_1 - err1);
  float desAcc2 = -(1/ timeConst_horizVel) * (estVelocity_2 - err2);
  float desRoll = -desAcc2 / gravity;
  float desPitch = -desAcc1 / gravity;
  float desYaw = 0;

  // vertical control
  desHeight = 1.5f;
  if ((in.userInput.buttonGreen == true)|((pos1<(des1+0.2f))&(pos1>(des1-0.3f))))
    { desHeight  = 0.0f; };
  const float desAcc3 = -2*dampingRatio_height*natFreq_height * estVelocity_3 - natFreq_height *natFreq_height*(estHeight-desHeight);

  float desNormalizedAcceleration = (gravity + desAcc3)/(cosf(estRoll)*cosf(estPitch));

   //angle controller
  cmdAngVel.x = -(1.0f/timeConstant_rollAngle)*(estRoll - desRoll);
  cmdAngVel.y = -(1.0f/timeConstant_pitchAngle)*(estPitch - desPitch);
  cmdAngVel.z = -(1.0f/timeConstant_yawAngle)*(estYaw - desYaw);

  //rates controller for angular
  cmdAngAcc.x = -(1.0f/timeConstant_rollRate)*(rateGyro_corr.x - cmdAngVel.x);
  cmdAngAcc.y = -(1.0f/timeConstant_pitchRate)*(rateGyro_corr.y - cmdAngVel.y);
  cmdAngAcc.z = -(1.0f/timeConstant_yawRate)*(rateGyro_corr.z - cmdAngVel.z);

  //adding the controlled values data to csv
  /* commented out for lab 5
  outVals.telemetryOutputs_plusMinus100[3] = cmdAngAcc.x;
  outVals.telemetryOutputs_plusMinus100[4] = cmdAngAcc.y;
  outVals.telemetryOutputs_plusMinus100[5] = cmdAngAcc.z;
  outVals.telemetryOutputs_plusMinus100[6] = cmdAngVel.x;
  outVals.telemetryOutputs_plusMinus100[7] = cmdAngVel.y;
  outVals.telemetryOutputs_plusMinus100[8] = cmdAngVel.z;
  */



  //define cn vector
  float n1 = (13.0f*estPitch*estYaw + 16.0f*cmdAngAcc.x)/1000000.0f;
  float n2 = (-13.0f*estRoll*estYaw + 16.0f*cmdAngAcc.y)/1000000.0f;
  float n3 = (29.0f*cmdAngAcc.z)/1000000.0f;
  float cS = mass*desNormalizedAcceleration; //force from 8m/s^2
  float cn[4] = {cS, n1, n2, n3};

  //mixer matrix multiplication
  for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 1; j++) {
                cp[i] = 0;
                for (int k = 0; k < 4; k++) {
                    cp[i] += M[i][k] * cn[k];
                }
            }
    }

//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y


  if (in.userInput.buttonBlue == true)
  {
      des1 = des1;
  } else {

      desired_angle.y = 0;
  };

  // assign vertical estimator values
  outVals.telemetryOutputs_plusMinus100[3] =  estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] =  estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] =  estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] =  estHeight;

  // assign desired roll and pitch angles
  outVals.telemetryOutputs_plusMinus100[7] =  desRoll;
  outVals.telemetryOutputs_plusMinus100[8] =  desPitch;

  // des normalized thrust output
  outVals.telemetryOutputs_plusMinus100[9] =  desNormalizedAcceleration;

  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;



      outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(cp[0]));
      outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(cp[1]));
      outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(cp[2]));
      outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(cp[3]));


  return outVals;


}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Accelerometer measurement

  printf("Acc:");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  printf("\n");
  //Rate gyro measurement
  printf("Gyro:");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");

  printf("Gyro bias:");
  printf("bias x=%6.3f, ", double(estGyroBias.x));
  printf("bias y=%6.3f, ", double(estGyroBias.y));
  printf("bias z=%6.3f, ", double(estGyroBias.z));
  printf("\n");

  printf("Gyro without bias:");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x - estGyroBias.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y - estGyroBias.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z - estGyroBias.z));
  printf("\n");

  printf("estimated altitude: %6.3f \n", double(estPitch));
  printf("estimated roll: %6.3f \n", double(estRoll));

  printf("Example variable values:\n");
  printf("  exampleVariable_int = %d\n", exampleVariable_int);

  printf("test variable values:\n");
  printf("  c1*10^6 = %6.3f\n", cp[0]*100000);
  printf("  c2*10^6 = %6.3f\n", cp[1]*100000);
  printf("  c3*10^6 = %6.3f\n", cp[2]*100000);
  printf("  c4*10^6 = %6.3f\n", cp[3]*100000);

  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
         double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
         double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.userInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.userInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.userInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.userInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.userInput.buttonArm)
    printf("buttonArm ");
  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor commands: = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
         double(lastMainLoopOutputs.motorCommand1),
         double(lastMainLoopOutputs.motorCommand2),
         double(lastMainLoopOutputs.motorCommand3),
         double(lastMainLoopOutputs.motorCommand4));

  printf("Last_range_=_%6.3fm,_", double(lastMainLoopInputs.heightSensor.value));

  printf("Last_flow:_x=%6.3f,_y=%6.3f\n",double(lastMainLoopInputs.opticalFlowSensor.value_x),double(lastMainLoopInputs.opticalFlowSensor.value_y));
  printf("error1: %6.3f\n",err1);
  printf("error2: %6.3f\n",err2);
  printf("pos1: %6.3f\n",pos1);
  printf("pos2: %6.3f\n",pos2);
  printf("est1: %6.3f\n",estVelocity_1);
  printf("est2: %6.3f\n",estVelocity_2);
  printf("est1*dt: %6.3f\n",estVelocity_1*dt);

}
