#define akp 0.4
#define aki 0.3
#define akd 0.8
#define akm 28 //Inches to degrees
#define akt 3.5  // Degrees of wheel turn to whole bot turn (Both sides must move, one side moves inverted)

void near();
void far();
void full();
void calibration();
void autonPID(void* ignore);
