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

vex::motor intake = motor(PORT12);

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
        Controller.Screen.setCursor(0,3);
        Controller.Screen.print("Battery: %damps", Brain.Battery.current());

        this_thread::sleep_for(100);
    }
    
}

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

                cascade_group.stop(brake);

            }
        }
        this_thread::sleep_for(10);
    }
    
}

void banch(){
    BanchControlMotor.setBrake(brake);

    BanchMotor.setVelocity(100, percent);

    while (true)
    {
        if(banch_in){
            BanchMotor.spin(forward);
        }else{
            if(banch_out){
                BanchMotor.spin(reverse);
            }else{
                BanchMotor.stop();
            }
        }
        this_thread::sleep_for(10);
    }
    
}

float banchArmDiff = 0.0;
void banch90(){
    BanchControlMotor.setVelocity(30, percent);
    BanchControlMotor.spinToPosition(320, degrees, true);

    return;
}

void banch110(){
    BanchControlMotor.setVelocity(30, percent);
    BanchControlMotor.spinToPosition(390, degrees, true);

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
        Controller.Screen.print( "%.2f", inertialSensor.angle(deg));

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

    bool hasPin = true;
    bool hasCup = false;
    gg_AI.modelDetection(true);
    
    while (!gg_AI.installed()){
        Controller.rumble("-.-");
        wait(500, msec);
    }

    Controller.Screen.print("AUTON", 50, 400);
    


    while(true){

        gg_AI.takeSnapshot(vex::aivision::ALL_TAGS);

        if(gg_AI.objectCount > 0){
            Controller.rumble(".");
        }

        wait(100, msec);

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
