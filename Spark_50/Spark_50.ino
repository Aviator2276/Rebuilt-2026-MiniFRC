/**
 * Example code for a robot using a NoU3 controlled with PestoLink: https://pestol.ink
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

// If your robot has more than a drivetrain, add those actuators here 
NoU_Motor frontLeftMotor(3);
NoU_Motor frontRightMotor(6);
NoU_Motor rearLeftMotor(1);
NoU_Motor rearRightMotor(4);

NoU_Motor intakeMotor(5);


NoU_Motor leftShooter(2);
NoU_Motor rightShooter(7);

NoU_Motor indexerMotor(8);

// Intake toggle: press button 0 to turn the intake on, press again to turn it off
bool intakeOn = false;
bool intakeButtonWasHeld = false;

// Shooter toggle: press button 1 to spin up the shooter, press again to stop it.
// When toggled on, the shooter runs at 100% for SHOOTER_SPINUP_MS,
// then drops to SHOOTER_TUNED_POWER (adjust this once you find the right value).
float SHOOTER_TUNED_POWER = 0.5;
unsigned long SHOOTER_SPINUP_MS = 1000;

bool shooterOn = false;
bool shooterButtonWasHeld = false;
unsigned long shooterStartTime = 0;

// This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

//The gyroscope sensor is by default precise, but not accurate. This is fixable by adjusting the angular scale factor.
//Tuning procedure: 
//Rotate the robot in place exactly 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process. 
float measured_angle = 31.416;
float angular_scale = (5.0*2.0*PI) / measured_angle;

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE
    PestoLink.begin("NoU3_Bluetooth");
    Serial.begin(115200);

    NoU3.begin();

    //rearRightMotor.setInverted(false);
    intakeMotor.setInverted(true);
    frontRightMotor.setInverted(true);

    NoU3.setServiceLight(LIGHT_CALIBRATING);
    NoU3.calibrateIMUs(); // this takes exactly one second. Do not move the robot during calibration.
}

void loop() {
    static unsigned long lastPrintTime = 0;
    if (lastPrintTime + 100 < millis()){
        Serial.printf("gyro yaw (radians): %.3f\r\n",  NoU3.yaw * angular_scale );
        lastPrintTime = millis();
    }

    // This measures your batterys voltage and sends it to PestoLink
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    if (PestoLink.isConnected()) {
        float fieldPowerX = PestoLink.getAxis(0);
        float fieldPowerY = -1 * PestoLink.getAxis(1);
        float rotationPower = -1 * PestoLink.getAxis(2);

        // Get robot heading (in radians) from the gyro
        float heading = NoU3.yaw * angular_scale;

        // Rotate joystick vector to be robot-centric
        float cosA = cos(heading);
        float sinA = sin(heading);

        float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
        float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

        //set motor power
        drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);

        // Toggle the intake when button 0 is pressed (not held)
        bool intakeButtonHeld = PestoLink.buttonHeld(0);
        if (intakeButtonHeld && !intakeButtonWasHeld) {
            intakeOn = !intakeOn;
        }
        intakeButtonWasHeld = intakeButtonHeld;

        if (intakeOn) {
            intakeMotor.set(1.0);
        } else {
            intakeMotor.set(0.0);
        }

        // Toggle the shooter when button 1 is pressed (not held)
        bool shooterButtonHeld = PestoLink.buttonHeld(1);
        if (shooterButtonHeld && !shooterButtonWasHeld) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                shooterStartTime = millis();
            }
        }
        shooterButtonWasHeld = shooterButtonHeld;

        if (shooterOn) {
            if (millis() - shooterStartTime < SHOOTER_SPINUP_MS) {
                // Spin up at full power first
                leftShooter.set(1.0);
                rightShooter.set(1.0);
            } else {
                // Then settle at the tuned shooting power
                leftShooter.set(SHOOTER_TUNED_POWER);
                rightShooter.set(SHOOTER_TUNED_POWER);
                
            }
        } else {
            leftShooter.set(0.0);
            rightShooter.set(0.0);
        }

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.stopMotors();
        NoU3.setServiceLight(LIGHT_DISABLED);
        intakeOn = false;
        intakeButtonWasHeld = false;
        shooterOn = false;
        shooterButtonWasHeld = false;
    }
}
