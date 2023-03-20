#include "main.h"
Motor leftFM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
Motor leftBack(leftBackPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
Motor rightBack(rightBackPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
Motor feedLeft(feedLeftPort,  E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_DEGREES);
Motor feedRight(feedRightPort,  E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_DEGREES);
ADIEncoder leftEnc(leftEncT, leftEncB, false);
ADIEncoder rightEnc(rightEncT, rightEncB, true);
Motor catapult(catapultPort,  E_MOTOR_GEAR_RED, true, E_MOTOR_ENCODER_DEGREES);

double targLeft, targRight, posLeft, posRight, prevPosLeft, prevPosRight, errorLeft, errorRight, prevErrorLeft, prevErrorRight, integLeft = 0, integRight = 0, derivLeft, derivRight, leftSpeed, rightSpeed;
bool targReach, voltControl = false;




// TUNE ROLLER TIMINGS AT SAIS





void autonPID(void* ignore){
  leftEnc.reset();
  rightEnc.reset();

  while (true /*&& competition::is_autonomous()*/) {
    if (voltControl){
      // printf("Voltage control\n");
      delay(20);
    } else {
      posLeft = leftEnc.get_value();
      posRight = rightEnc.get_value();

      errorLeft = targLeft - posLeft; //Should I get avg of leftFM and leftBack instead
      errorRight = targRight - posRight; //Same here

      prevErrorLeft = targLeft - prevPosLeft;
      prevErrorRight = targRight - prevPosRight;
      
      integLeft += errorLeft;
      integRight += errorRight;

      if(errorLeft == 0){
        integLeft = 0;
      } else if (integLeft>=50){
        integLeft = 50;
      } else if (integLeft<=-50){
        integLeft = -50;
      }

      if(errorRight == 0){
        integRight = 0;
      } else if (integLeft>=50){
        integRight = 50;
      } else if (integRight<=-50){
        integRight = -50;
      }
      
      derivLeft = errorLeft - prevErrorLeft;
      derivRight = errorRight - prevErrorRight;

      leftSpeed = errorLeft*akp + integLeft*aki + derivLeft*akd;
      rightSpeed = errorRight*akp + integRight*aki + derivRight*akd;

      leftFM.move(leftSpeed);
      leftBack.move(leftSpeed);
      rightFM.move(rightSpeed);
      rightBack.move(rightSpeed);

      prevPosLeft = posLeft;
      prevPosRight = posRight;

      // prevErrorLeft = errorLeft;
      // prevErrorRight = errorRight;

      targReach = fabs(errorLeft)<10 && fabs(errorRight)<10 /*&& isError()*/;

      // printf("A    Left: %f %f    Right: %f %f    targReach: %s     CataError: %s   Integs: %f %f\n", errorLeft, leftSpeed, errorRight, rightSpeed, (targReach)? "true" : "false", (isError())? "true" : "false", integLeft, integRight);
      delay(20);
    }
  }
  printf("ENDED");
}

void move(double inches){
  targLeft += inches*akm;
  targRight += inches*akm;

  printf("move %f\n", inches);
  delay(50);
  while (true){
    if (targReach){
      delay(100);
      printf("Move done\n");
      return;
    }
    delay(50);
  }
}

void turn(double degrees){
  targLeft += degrees*akt;
  targRight -= degrees*akt;

  delay(50);
  while (true){
    if (targReach){
      delay(100);
      printf("Turn done\n");
      return;
    }
    delay(50);
  }
}

void near(){ //Starts at rollers
  leftFM.move(30);
  leftBack.move(30);
  rightFM.move(30);
  rightBack.move(30);
  delay(1500);
  feedLeft.move(60);
  feedRight.move(60); //Does the roller, base moves to apply pressure on roller
  delay(215);
  feedLeft.move(0);
  feedRight.move(0);
  leftFM.move(0);
  leftBack.move(0);
  rightFM.move(0);
  rightBack.move(0); //Stop
}

void far(){
  Task autonPIDTask (autonPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "autonPIDTask");
  
  move(24);
  
  turn(90);

  autonPIDTask.remove();
  printf("Task removed\n");

  leftFM.move(30);
  leftBack.move(30);
  rightFM.move(30);
  rightBack.move(30);
  delay(1500);
  feedLeft.move(60);
  feedRight.move(60); //Does the roller, base moves to apply pressure on roller
  delay(215);
  feedLeft.move(0);
  feedRight.move(0);
  leftFM.move(0);
  leftBack.move(0);
  rightFM.move(0);
  rightBack.move(0); //Stop
}

void full(){
  leftFM.move(30);
  leftBack.move(30);
  rightFM.move(30);
  rightBack.move(30);
  delay(1500);
  feedLeft.move(60);
  feedRight.move(60); //Does the roller, base moves to apply pressure on roller
  delay(430);
  feedLeft.move(0);
  feedRight.move(0);
  leftFM.move(0);
  leftBack.move(0);
  rightFM.move(0);
  rightBack.move(0); //Stop

  Task autonPIDTask (autonPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "autonPIDTask");

  move(-18);
  turn(90);
  move(20);
  
  voltControl = true;
  
  leftFM.move(30);
  leftBack.move(30);
  rightFM.move(30);
  rightBack.move(30);
  delay(1500);
  feedLeft.move(60);
  feedRight.move(60); //Does the roller, base moves to apply pressure on roller
  delay(430);
  feedLeft.move(0);
  feedRight.move(0);
  leftFM.move(0);
  leftBack.move(0);
  rightFM.move(0);
  rightBack.move(0); //Stop

  voltControl = false;

  turn(-75);
  move(-48);

  catapultShoot();

  autonPIDTask.remove();
}

void calibration(){
  Task autonPIDTask (autonPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "autonPIDTask");

  // move(24);
  voltControl = true;
  delay(50);
  leftFM.move(70);
  delay(2000);
  voltControl = false;

  move(0);

  feedLeft.move(0);
  feedRight.move(0);
  leftFM.move(0);
  leftBack.move(0);
  rightFM.move(0);
  rightBack.move(0); //Stop

  autonPIDTask.remove();
  printf("Task removed\n");
}
