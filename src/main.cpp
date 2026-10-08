#include "vex.h"
#include "string.h"

using namespace vex;

//CONFIG

float CONFIG_DEADZONE = 10;

//END

vex::competition Competition;

vex::brain Brain;
vex::controller Controller = controller();

vex::motor f_leftMotor = motor(PORT20, false);
vex::motor f_rightMotor = motor(PORT11, true);
vex::motor b_leftMotor = motor(PORT10, false);
vex::motor b_rightMotor = motor(PORT1, true);

vex::motor_group left_mg = motor_group(f_leftMotor, b_leftMotor);
vex::motor_group right_mg = motor_group(f_rightMotor, b_rightMotor);

drivetrain gg_drivetrain = drivetrain(
    left_mg,  // left motor object
    right_mg, // right motor object
    259.34,     // wheelTravel
    320,        // trackWidth
    40,         // wheelBase
    mm,         // unit
    0.75 );      // externalGearRatio

// Brain screen resolution is 480px x 240px

int CUR_HEIGHT = 0; // 0 = level, 1 = next, 2 = last

void gg_CascadeRaise(){
    switch (CUR_HEIGHT)
    {
        case 0:
            break;
        case 1:
            break;
        case 2:
            break;
        
        default:
            break;
    }
}

void drive(){
    while (true)
    {
        float a3 = Controller.Axis3.value();
        float a1 = Controller.Axis1.value();

        float drivePower = abs(a3) > CONFIG_DEADZONE ? a3 : 0.0f;
        float steerPower = abs(a1) > CONFIG_DEADZONE ? a1 : 0.0f;

        gg_drivetrain.arcade(-drivePower / 2.0, -steerPower / 2, percent);

        this_thread::sleep_for(10);
    }
    
}

void autonomous(){
    //github test
    vex::aivision gg_AI = aivision(5);

    bool hasPin = true;
    bool hasCup = false;

    Controller.Screen.print("AUTON", 50, 400);
    gg_AI.tagDetection(true);

    while (true)
    {
        
        if(hasPin){
            gg_AI.takeSnapshot(aivision::ALL_TAGS);
            wait(0.2, sec);
            Controller.Screen.clearScreen();

            Controller.Screen.print( __STRING(gg_AI.objects.getLength()) , 50, 100);
            while(gg_AI.objects.getLength() == 0){
                Controller.rumble(".");
                gg_drivetrain.turn(left);

            }
            gg_drivetrain.stop();

            gg_AI.takeSnapshot(aivision::ALL_AIOBJS);
            for (size_t i = 0; i < gg_AI.objects.getLength(); i++)
            {
                aivision::object obj = gg_AI.objects[i];
                if(obj.id == 3 or obj.id == 2){
                    //this means it is a red goal
                    Controller.rumble("..");
                    if(gg_AI.objects[0].centerX < 160){
                        while (gg_AI.objects[0].centerX != 160) {
                            gg_AI.takeSnapshot(aivision::ALL_AIOBJS);
                            gg_drivetrain.turn(left);
                            wait(5, msec);
                        }
                        gg_drivetrain.stop();
                        
                    

                    }
                
                    if(gg_AI.objects[0].centerX > 160){
                        while (gg_AI.objects[0].centerX != 160) {
                            gg_AI.takeSnapshot(aivision::ALL_AIOBJS);
                            gg_drivetrain.turn(right);
                            wait(5, msec);
                        }
                        gg_drivetrain.stop(brake);
                    }
                }
            }
        }

        wait(5, msec);
    }
    
}

int main() {

    Controller.rumble("..");

    Competition.autonomous(autonomous);
    Competition.drivercontrol(drive);
   
    while(true) {
        this_thread::sleep_for(10);
    }
}
