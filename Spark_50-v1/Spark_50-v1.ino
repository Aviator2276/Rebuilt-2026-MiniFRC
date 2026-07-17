/**
 * Robot using a NoU3 controlled with PestoLink.
 *
 * Controls:
 *   Button 0:
 *     Hold to shoot.
 *
 *   Button 1:
 *     Hold to run the intake.
 *     Release to stop the intake.
 *
 * Stepped-shot sequence:
 *   1. Indexer runs for INDEXER_STEP_MS.
 *   2. Shooter remains at base power while feeding.
 *   3. Indexer stops.
 *   4. Shooter runs at 100% for BOOST_MS.
 *   5. Shooter returns to base power for SETTLE_MS.
 *   6. Sequence repeats while Button 0 remains held.
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

// --------------------------------------------------
// Motors
// --------------------------------------------------

NoU_Motor frontLeftMotor(3);
NoU_Motor frontRightMotor(6);
NoU_Motor rearLeftMotor(1);
NoU_Motor rearRightMotor(4);

NoU_Motor intakeMotor(5);

NoU_Motor leftShooter(2);
NoU_Motor rightShooter(7);

NoU_Motor indexerMotor(8);

NoU_Drivetrain drivetrain(
    &frontLeftMotor,
    &frontRightMotor,
    &rearLeftMotor,
    &rearRightMotor
);

// --------------------------------------------------
// Gyroscope calibration
// --------------------------------------------------

float measured_angle = 31.416f;

float angular_scale =
    (5.0f * 2.0f * PI) / measured_angle;

// --------------------------------------------------
// Controller buttons
// --------------------------------------------------

constexpr uint8_t SHOOT_BUTTON = 0;
constexpr uint8_t INTAKE_BUTTON = 1;

// --------------------------------------------------
// Intake configuration
// --------------------------------------------------

constexpr float INTAKE_POWER = 1.00f;

// --------------------------------------------------
// Shooter configuration
// --------------------------------------------------

/*
 * true:
 *   Timed shots with recovery between each ball.
 *
 * false:
 *   Indexer runs continuously while Button 0 is held.
 *   Recovery begins when Button 0 is released.
 */
constexpr bool ENABLE_STEPPED_SHOTS = true;

constexpr float SHOOTER_BASE_POWER = 0.50f;
constexpr float SHOOTER_BOOST_POWER = 1.00f;
constexpr float INDEXER_POWER = 1.00f;

constexpr uint32_t INDEXER_STEP_MS = 250;
constexpr uint32_t BOOST_MS = 150;
constexpr uint32_t SETTLE_MS = 850;

// --------------------------------------------------
// Stepped-shot state
// --------------------------------------------------

enum class ShotState {
    IDLE,
    FEEDING,
    BOOSTING,
    SETTLING
};

ShotState shotState = ShotState::IDLE;
uint32_t shotStateStartedAt = 0;

// --------------------------------------------------
// Continuous-mode recovery state
// --------------------------------------------------

enum class RecoveryState {
    NONE,
    BOOSTING,
    SETTLING
};

RecoveryState continuousRecoveryState =
    RecoveryState::NONE;

bool previousContinuousShootButton = false;
uint32_t continuousStateStartedAt = 0;

// --------------------------------------------------
// Utility functions
// --------------------------------------------------

bool hasElapsed(
    uint32_t now,
    uint32_t startTime,
    uint32_t duration
) {
    return static_cast<uint32_t>(
        now - startTime
    ) >= duration;
}

void setShooterPower(float power) {
    leftShooter.set(power);
    rightShooter.set(power);
}

// --------------------------------------------------
// Intake control
// --------------------------------------------------

void updateIntake(bool intakeButtonHeld) {
    if (intakeButtonHeld) {
        intakeMotor.set(INTAKE_POWER);
    } else {
        intakeMotor.set(0.0f);
    }
}

// --------------------------------------------------
// Shooter state helpers
// --------------------------------------------------

void enterShotState(
    ShotState newState,
    uint32_t now
) {
    shotState = newState;
    shotStateStartedAt = now;
}

void enterContinuousRecoveryState(
    RecoveryState newState,
    uint32_t now
) {
    continuousRecoveryState = newState;
    continuousStateStartedAt = now;
}

void resetShotSequence() {
    shotState = ShotState::IDLE;
    shotStateStartedAt = 0;

    continuousRecoveryState =
        RecoveryState::NONE;

    previousContinuousShootButton = false;
    continuousStateStartedAt = 0;
}

// --------------------------------------------------
// Continuous indexer mode
// --------------------------------------------------

void updateContinuousShooter(
    bool shootButtonHeld,
    uint32_t now
) {
    /*
     * Start recovery when the shoot button is released
     * and the indexer stops.
     */
    if (
        !shootButtonHeld &&
        previousContinuousShootButton
    ) {
        enterContinuousRecoveryState(
            RecoveryState::BOOSTING,
            now
        );
    }

    if (shootButtonHeld) {
        /*
         * Run the indexer continuously while keeping
         * the shooter at its normal distance power.
         */
        continuousRecoveryState =
            RecoveryState::NONE;

        indexerMotor.set(INDEXER_POWER);
        setShooterPower(SHOOTER_BASE_POWER);

    } else {
        indexerMotor.set(0.0f);

        switch (continuousRecoveryState) {
            case RecoveryState::NONE: {
                setShooterPower(
                    SHOOTER_BASE_POWER
                );

                break;
            }

            case RecoveryState::BOOSTING: {
                setShooterPower(
                    SHOOTER_BOOST_POWER
                );

                if (
                    hasElapsed(
                        now,
                        continuousStateStartedAt,
                        BOOST_MS
                    )
                ) {
                    enterContinuousRecoveryState(
                        RecoveryState::SETTLING,
                        now
                    );
                }

                break;
            }

            case RecoveryState::SETTLING: {
                setShooterPower(
                    SHOOTER_BASE_POWER
                );

                if (
                    hasElapsed(
                        now,
                        continuousStateStartedAt,
                        SETTLE_MS
                    )
                ) {
                    continuousRecoveryState =
                        RecoveryState::NONE;
                }

                break;
            }
        }
    }

    previousContinuousShootButton =
        shootButtonHeld;
}

// --------------------------------------------------
// Stepped-shot mode
// --------------------------------------------------

void updateSteppedShooter(
    bool shootButtonHeld,
    uint32_t now
) {
    switch (shotState) {
        case ShotState::IDLE: {
            indexerMotor.set(0.0f);
            setShooterPower(
                SHOOTER_BASE_POWER
            );

            if (shootButtonHeld) {
                enterShotState(
                    ShotState::FEEDING,
                    now
                );
            }

            break;
        }

        case ShotState::FEEDING: {
            /*
             * Feed one ball while the shooter remains
             * at its normal distance-setting power.
             */
            indexerMotor.set(
                INDEXER_POWER
            );

            setShooterPower(
                SHOOTER_BASE_POWER
            );

            if (
                hasElapsed(
                    now,
                    shotStateStartedAt,
                    INDEXER_STEP_MS
                )
            ) {
                indexerMotor.set(0.0f);

                enterShotState(
                    ShotState::BOOSTING,
                    now
                );
            }

            break;
        }

        case ShotState::BOOSTING: {
            /*
             * The indexer is stopped while the shooter
             * runs at full power to recover speed.
             */
            indexerMotor.set(0.0f);

            setShooterPower(
                SHOOTER_BOOST_POWER
            );

            if (
                hasElapsed(
                    now,
                    shotStateStartedAt,
                    BOOST_MS
                )
            ) {
                enterShotState(
                    ShotState::SETTLING,
                    now
                );
            }

            break;
        }

        case ShotState::SETTLING: {
            /*
             * Return to base power and wait before
             * feeding another ball.
             */
            indexerMotor.set(0.0f);

            setShooterPower(
                SHOOTER_BASE_POWER
            );

            if (
                hasElapsed(
                    now,
                    shotStateStartedAt,
                    SETTLE_MS
                )
            ) {
                if (shootButtonHeld) {
                    enterShotState(
                        ShotState::FEEDING,
                        now
                    );
                } else {
                    enterShotState(
                        ShotState::IDLE,
                        now
                    );
                }
            }

            break;
        }
    }
}

// --------------------------------------------------
// Shooter mode selection
// --------------------------------------------------

void updateShooterAndIndexer(
    bool shootButtonHeld,
    uint32_t now
) {
    if (ENABLE_STEPPED_SHOTS) {
        updateSteppedShooter(
            shootButtonHeld,
            now
        );
    } else {
        updateContinuousShooter(
            shootButtonHeld,
            now
        );
    }
}

// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup() {
    PestoLink.begin("50 - Spark");
    Serial.begin(115200);

    NoU3.begin();

    // Your tested inversion settings.
    rearRightMotor.setInverted(false);
    intakeMotor.setInverted(true);
    frontRightMotor.setInverted(true);

    NoU3.setServiceLight(
        LIGHT_CALIBRATING
    );

    NoU3.calibrateIMUs();
}

// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop() {
    static uint32_t lastPrintTime = 0;

    const uint32_t now = millis();

    if (
        hasElapsed(
            now,
            lastPrintTime,
            100
        )
    ) {
        Serial.printf(
            "gyro yaw (radians): %.3f\r\n",
            NoU3.yaw * angular_scale
        );

        lastPrintTime = now;
    }

    const float batteryVoltage =
        NoU3.getBatteryVoltage();

    PestoLink.printBatteryVoltage(
        batteryVoltage
    );

    if (PestoLink.isConnected()) {
        // ------------------------------------------
        // Drivetrain
        // ------------------------------------------

        const float fieldPowerX =
            PestoLink.getAxis(0);

        const float fieldPowerY =
            -1.0f * PestoLink.getAxis(1);

        const float rotationPower =
            -1.0f * PestoLink.getAxis(2);

        // Retained for later gyro testing.
        const float heading =
            NoU3.yaw * angular_scale;

        const float cosA = cos(heading);
        const float sinA = sin(heading);

        const float robotPowerX =
            fieldPowerX * cosA +
            fieldPowerY * sinA;

        const float robotPowerY =
            -fieldPowerX * sinA +
            fieldPowerY * cosA;

        /*
         * Intentionally retain your existing drivetrain
         * behavior until gyro control is tested.
         */
        drivetrain.holonomicDrive(
            fieldPowerX,
            fieldPowerY,
            rotationPower
        );

        (void)robotPowerX;
        (void)robotPowerY;

        NoU3.setServiceLight(
            LIGHT_ENABLED
        );

        // ------------------------------------------
        // Intake — hold Button 1
        // ------------------------------------------

        const bool intakeButtonHeld =
            PestoLink.buttonHeld(
                INTAKE_BUTTON
            );

        updateIntake(
            intakeButtonHeld
        );

        // ------------------------------------------
        // Shooter — hold Button 0
        // ------------------------------------------

        const bool shootButtonHeld =
            PestoLink.buttonHeld(
                SHOOT_BUTTON
            );

        updateShooterAndIndexer(
            shootButtonHeld,
            now
        );

    } else {
        NoU3.stopMotors();

        NoU3.setServiceLight(
            LIGHT_DISABLED
        );

        resetShotSequence();
    }
}