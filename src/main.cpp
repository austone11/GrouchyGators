#include "vex.h"
#include "string.h"

using namespace vex;

//CONFIG

float CONFIG_DEADZONE = 10;
float CONFIG_CAS_MAX = 10000;
float CONFIG_CAS_MIN = 0;
float CONFIG_DRIVE_DIV = 1.5;

//END

vex::competition Competition;

vex::brain Brain;
vex::controller Controller = controller();

vex::motor f_leftMotor = motor(PORT1, false);
vex::motor f_rightMotor = motor(PORT10, true);

vex::motor b_rightMotor = motor(PORT20, true);
vex::motor b_leftMotor = motor(PORT11, false);

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

vex::motor cas_lift1 = motor(PORT3, false);
vex::motor cas_lift2 = motor(PORT9, true);
vex::motor_group cascade_group = motor_group(cas_lift1, cas_lift2);

vex::motor intake = motor(PORT2);

inertial inertialSensor = inertial(
    PORT6,
    right);


bool banch_up = false;
bool banch_down = false;
bool banch_in = false;
bool banch_out = false;
vex::motor BanchControlMotor = motor(PORT7); //11
vex::motor BanchMotor = motor(PORT8); // 12



//r1 down
//r2 up

bool cas_for = false;
bool cas_rev = false;

void customDisplay(){
    while (true)
    {
        //Controller.Screen.setCursor(0,3);
        //Controller.Screen.print("Battery: %damps", Brain.Battery.current());

        this_thread::sleep_for(100);
    }
    
}

bool ignoreStopCas = false;
void cascadeLift(){
    cascade_group.setVelocity(300, percent);
    
    while (true)
    {
        if(cas_for and cascade_group.position(degrees) <= CONFIG_CAS_MAX){
            cascade_group.spin(forward);

        }else{
            if (cas_rev and cascade_group.position(degrees) > CONFIG_CAS_MIN)
            {
                cascade_group.spin(reverse);
            }else{

                if(not ignoreStopCas){
                    cascade_group.stop(brake);
                }
                

            }
        }
        this_thread::sleep_for(10);
    }
    
}

void banch(){
    BanchControlMotor.setBrake(brake);

    BanchMotor.setVelocity(100, percent);
    intake.setVelocity(200, percent);

    while (true)
    {
        if(banch_in){
            BanchMotor.spin(forward);
            intake.spin(reverse);
        }else{
            if(banch_out){
                BanchMotor.spin(reverse);
                intake.spin(forward);
            }else{
                BanchMotor.stop();
                intake.stop();
            }
        }
        this_thread::sleep_for(10);
    }
    
}

float banchArmDiff = 0.0;
void banch90(){
    BanchControlMotor.setVelocity(30, percent);
    BanchControlMotor.spinToPosition(320, degrees, true);

    if(cascade_group.position(degrees) < 200){
        ignoreStopCas = true;
        cascade_group.spinFor(forward, 360, degrees, true);
        ignoreStopCas = false;
    }

    return;
}

void banch110(){
    BanchControlMotor.setVelocity(30, percent);
    BanchControlMotor.spinToPosition(390, degrees, true);

    if(cascade_group.position(degrees) < 200){
        ignoreStopCas = true;
        cascade_group.spinFor(forward, 360, degrees, true);
        ignoreStopCas = false;
    }

    return;
}

void banchHome(){
    BanchControlMotor.setVelocity(30, percent);
    BanchControlMotor.spinToPosition(0, degrees, true);
    BanchControlMotor.setPosition(0, degrees);

    return;
}

void spinTo(float target){
    
    float currentAngle = inertialSensor.angle(deg);


    return;
}

void debugSpin180(){
    spinTo( inertialSensor.angle(deg) - 180);
    return;
}

void drive(){
    //thread displayThread = thread(customDisplay);
    thread cascadeThread = thread(cascadeLift);
    thread banchThread = thread(banch);

    Controller.ButtonUp.pressed(banch90);
    Controller.ButtonDown.pressed(banchHome);
    Controller.ButtonRight.pressed(banch110);

    while (true)
    {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(0,0);
        //Controller.Screen.print( "%.2f", inertialSensor.angle(deg));

        cas_for = Controller.ButtonR1.pressing();
        cas_rev = Controller.ButtonR2.pressing();
        banch_in = Controller.ButtonL1.pressing();
        banch_out = Controller.ButtonL2.pressing();
        

        float a3 = Controller.Axis3.value();
        float a1 = Controller.Axis1.value();

        float drivePower = abs(a3) > CONFIG_DEADZONE ? a3 : 0.0f;
        if(cascade_group.position(degrees) > 2000){
            if(drivePower != 0){
                drivePower = drivePower - (cascade_group.position(degrees) / 1000);
            }
            
        }
        float steerPower = abs(a1) > CONFIG_DEADZONE ? a1 : 0.0f;

        gg_drivetrain.arcade( -drivePower / CONFIG_DRIVE_DIV, -steerPower / CONFIG_DRIVE_DIV, percent);

        this_thread::sleep_for(10);
    }
    
}

vex::aivision gg_AI = aivision(PORT5);
void autonomous(){
    //github test

    bool hasPin = true; // Start with a pin. 
    bool hasCup = false;

    //error = 160 - obj.centerX
    //turn_speed = 0.3 * error   # kP = 0.3

    gg_AI.modelDetection(true);
    
    while (!gg_AI.installed()){
        wait(10, msec);
    }

    Controller.Screen.print("AUTON", 50, 400);
    
    // Going to use P Controller
    /*
        Order:
            Start out with our pin.
            Search for the AprilTags of the goals.
            Locate, use P controller turn and move towards.
            Drop our pin.

            Set hasPin var to false.
            repeat and that should influence the next loop.

    */


    while(true){

        if(not hasPin){

            gg_AI.takeSnapshot(vex::aivision::ALL_AIOBJS); // Search for tags because we have a pin.
            while(gg_AI.objectCount <= 0){

                wait(20,msec);

                gg_AI.takeSnapshot(vex::aivision::ALL_AIOBJS);
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1,1);
                Controller.Screen.print('looking for objects');

            }


            if(gg_AI.objectCount > 0){

                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1,1);
                Controller.Screen.print(gg_AI.objects[0].className);

                gg_drivetrain.drive(reverse);

                intake.setVelocity(200, percent);
                BanchMotor.setVelocity(200, percent);
                intake.spin(reverse);

                BanchMotor.spin(forward);

            }else{
                BanchMotor.setVelocity(200, percent);
                gg_drivetrain.stop();
                intake.stop();

                BanchMotor.stop();
            }

        }

        if(hasPin){

            // Id 3 = Blue goal 2
            // Id 2 = Blue goal 1

            gg_AI.takeSnapshot(vex::aivision::ALL_TAGS); // Search for tags because we have a pin.
            while(gg_AI.objectCount <= 0){

                wait(20,msec);

                gg_AI.takeSnapshot(vex::aivision::ALL_TAGS);
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1,1);
                Controller.Screen.print('looking for objects');

            }

        }
    }
    
}

int main() {

    Controller.rumble("..");

    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating()) wait(20, msec);

    Competition.autonomous(autonomous);
    Competition.drivercontrol(drive);
   
    while(true) {
        this_thread::sleep_for(10);
    }
}
