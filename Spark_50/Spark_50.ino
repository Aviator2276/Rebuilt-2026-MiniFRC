/**
 * Robot using a NoU3 controlled with PestoLink: https://pestol.ink
 *
 * Controls:
 *   Button 0: toggle intake on/off
 *   Button 1: toggle shooter on/off (70% power)
 *   Button 2: hold to feed balls with the indexer
 *   Button 3: hold for shooter at max power (overrides the toggle)
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

NoU_Motor frontLeftMotor(3);
NoU_Motor frontRightMotor(6);
NoU_Motor rearLeftMotor(1);
NoU_Motor rearRightMotor(4);

NoU_Motor intakeMotor(5);

NoU_Motor leftShooter(2);
NoU_Motor rightShooter(7);

NoU_Motor indexerMotor(8);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

float SHOOTER_POWER = 0.5;

bool intakeOn = false;
bool intakeButtonWasHeld = false;

bool shooterOn = false;
bool shooterButtonWasHeld = false;

void setup() {
    PestoLink.begin("50 - Spark");
    Serial.begin(115200);

    NoU3.begin();

    intakeMotor.setInverted(true);
    frontRightMotor.setInverted(true);
}

void loop() {
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    // "voltage:<value>@<ms>" lines show up as a chart in the PestoLink terminal
    PestoLink.printfTerminal("voltage:%.2f@%lu", batteryVoltage, millis());

    if (PestoLink.isConnected()) {
        float powerX = PestoLink.getAxis(0);
        float powerY = -1 * PestoLink.getAxis(1);
        float rotationPower = -1 * PestoLink.getAxis(2);

        drivetrain.holonomicDrive(powerX, powerY, rotationPower);

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
        }
        shooterButtonWasHeld = shooterButtonHeld;

        // Hold button 3 for max shooter power, overriding the toggle
        if (PestoLink.buttonHeld(3)) {
            leftShooter.set(1.0);
            rightShooter.set(1.0);
        } else if (shooterOn) {
            leftShooter.set(SHOOTER_POWER);
            rightShooter.set(SHOOTER_POWER);
        } else {
            leftShooter.set(0.0);
            rightShooter.set(0.0);
        }

        // Hold button 2 to feed balls with the indexer
        if (PestoLink.buttonHeld(2)) {
            indexerMotor.set(1.0);
        } else {
            indexerMotor.set(0.0);
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
