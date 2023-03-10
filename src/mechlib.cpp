#include "main.h"

Controller master(E_CONTROLLER_MASTER);

double pos, targ = 0, error, prevError, deriv, power;
bool shoot;

void catapultPID(void *ignore) {
  Motor catapult(catapultPort, E_MOTOR_GEAR_RED, true, E_MOTOR_ENCODER_DEGREES);
  Rotation rotation(rotationPort, false);

  catapult.tare_position();

  error = prevError = targ - rotation.get_angle();
  deriv = error - prevError;

  catapult.move(error * kp + deriv * kd);
  // catapultRight.move(error"kp + deriv*kd)

  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      catapult.move(-115);
      // catapultRight.move(-115);
      targ = rotation.get_angle();
      printf("Manual\n");
    } else if (shoot){
      shoot = false;
      catapult.move(-127);
      // catapultRight.move(127);
      delay(1000);
    } else {
      pos = rotation.get_angle();
      error = targ - pos; //Porportion
      deriv = error - prevError; //Deriv
      power = error * kp + deriv * kd;

      catapult.move((power > -35)? 0 : power); //Move

      prevError = error;

      // printf("%f %f\n", error, error * kp + deriv * kd);
      // printf("Pos: %d Targ: %f\n", rotation.get_angle(), targ);
    }
  }
}

void catapultShoot() {
  shoot = true;
}

bool isError() {
  return -500 < error && error < 500;
}
