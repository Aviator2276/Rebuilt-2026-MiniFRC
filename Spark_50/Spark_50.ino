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

// Shooter tuning test: press button 2 to start (press again to abort).
// For each power in TEST_POWERS it rests (shooter off, battery recovers),
// then runs the shooter at that power while printing "time power voltage"
// to the PestoLink terminal. When the voltage stops dropping, the wheel
// has finished spinning up - that time is the spin-up time for that power.
float TEST_POWERS[] = {0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
int NUM_TEST_POWERS = 7;
unsigned long TEST_REST_MS = 2000;
unsigned long TEST_RUN_MS = 3000;

bool testRunning = false;
int testIndex = 0;
bool testResting = true;
unsigned long testStageStart = 0;
bool testButtonWasHeld = false;

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

// Runs one step of the shooter test. Call this every loop while the test is on.
void runShooterTest() {
    unsigned long now = millis();
    float power = TEST_POWERS[testIndex];

    if (testResting) {
        // Shooter off, let the battery voltage recover
        leftShooter.set(0.0);
        rightShooter.set(0.0);

        if (now - testStageStart >= TEST_REST_MS) {
            testResting = false;
            testStageStart = now;
        }
    } else {
        // Run at this test power and log time / power / voltage
        leftShooter.set(power);
        rightShooter.set(power);

        static unsigned long lastTestPrint = 0;
        if (now - lastTestPrint >= 200) {
            PestoLink.printfTerminal("%lu ms  %d%%  %.2f V",
                now - testStageStart, (int)(power * 100), NoU3.getBatteryVoltage());
            lastTestPrint = now;
        }

        if (now - testStageStart >= TEST_RUN_MS) {
            // Move on to the next power level
            testResting = true;
            testStageStart = now;
            testIndex++;
            if (testIndex >= NUM_TEST_POWERS) {
                testRunning = false;
                leftShooter.set(0.0);
                rightShooter.set(0.0);
            }
        }
    }
}

void loop() {
    static unsigned long lastPrintTime = 0;
    if (lastPrintTime + 100 < millis()){
        Serial.printf("gyro yaw (radians): %.3f\r\n",  NoU3.yaw * angular_scale );

        // This shows up in the terminal on the PestoLink web app
        //PestoLink.printfTerminal("yaw: %.2f | intake: %s | shooter: %s",
        //    NoU3.yaw * angular_scale,
        //    intakeOn ? "ON" : "off",
        //    shooterOn ? "ON" : "off");

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

        //set motor power (raw joystick values; switch back to robotPowerX/Y once the gyro is tuned)
        drivetrain.holonomicDrive(fieldPowerX, fieldPowerY, rotationPower);

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

        // Start or abort the shooter tuning test when button 2 is pressed
        bool testButtonHeld = PestoLink.buttonHeld(2);
        if (testButtonHeld && !testButtonWasHeld) {
            testRunning = !testRunning;
            testIndex = 0;
            testResting = true;
            testStageStart = millis();
            shooterOn = false;
        }
        testButtonWasHeld = testButtonHeld;

        // Toggle the shooter when button 1 is pressed (not held)
        bool shooterButtonHeld = PestoLink.buttonHeld(1);
        if (shooterButtonHeld && !shooterButtonWasHeld) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                shooterStartTime = millis();
            }
        }
        shooterButtonWasHeld = shooterButtonHeld;

        if (testRunning) {
            runShooterTest();
        } else if (shooterOn) {
            if (millis() - shooterStartTime < SHOOTER_SPINUP_MS) {
                // Spin up at full power first
                leftShooter.set(1.0);
                rightShooter.set(1.0);
            } else {
                // Then settle at the tuned shooting power
                leftShooter.set(SHOOTER_TUNED_POWER);
                rightShooter.set(SHOOTER_TUNED_POWER);
                PestoLink.printfTerminal("Hello world");
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
        testRunning = false;
        testButtonWasHeld = false;
    }
}
