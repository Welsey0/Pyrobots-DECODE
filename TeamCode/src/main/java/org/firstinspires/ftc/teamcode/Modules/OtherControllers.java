package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

@Config
public class OtherControllers {
    private MDS mds = null;
    public void setMDS(MDS mds) {
        this.mds = mds;
    }
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public OtherControllers(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    long currentTime = System.currentTimeMillis();
    public boolean firstCycle = true;
    public Gamepad pgamepad1 = new Gamepad();
    public Gamepad pgamepad2 = new Gamepad();

    public void init() {
        pgamepad1.copy(gamepad1);
        pgamepad2.copy(gamepad2);
    }


    // lift controller vars
    double liftDirection = -1; // should be -1 or 1.
    double backPower = 0;
    double frontPower = 0;
    public int startslidesup = 0;

    // lasts (lift controller)
    double backPowerLast = backPower;
    double frontPowerLast = frontPower;

    // lift controller
    public void liftController(double y) {
        // apply lift motor state
        if (gamepad2.right_stick_button) {
            // manual control mode
            backPower = y;
            frontPower = y;
        } else {
            if (y == 0) {
                if (liftDirection == -1) {
                    backPower = -0.1;
                    frontPower = -0.1;
                } else if (liftDirection == 1) {
                    backPower = 0.1;
                    frontPower = 0.1;
                }
            } else if (y > 0) {
                // self-right bot based on hub imu yaw
                liftDirection = 1;
                YawPitchRollAngles robotOrientation;
                robotOrientation = mds.hardware.imu.getRobotYawPitchRollAngles();
                double pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
                if (pitch > -1) {
                    backPower = y + ((pitch * -1 )* 0.25);
                } else {
                    backPower = y; //negative pitch
                }
                if (pitch < 1) {
                    frontPower = y + (pitch * 0.5);
                } else {
                    frontPower = y; //positive pitch
                }
            } else if (y < 0) {
                liftDirection = -1;
                backPower = y;
                frontPower = y;
            }
        }

        if(startslidesup < 5){
            frontPower = -1;
            backPower = -1;
            startslidesup++;
        }

        // apply lift motor state if it is different
        if ((backPowerLast != backPower || frontPowerLast != frontPower) || firstCycle) {
            mds.hardware.slideBack.setPower(backPower);
            mds.hardware.slideFront.setPower(frontPower);
        }

        backPowerLast = backPower;
        frontPowerLast = frontPower;

        // slides pullup at beginning of opmode
    }

    // turret controller vars
    public static String trTarget;
    public static boolean trLoveAtFirstSight = true;
    public boolean trTargetFound;
    public String trMode = "none";
    AprilTagDetection trDetection;
    public static double trSens = 10;
    public static double trIntens = 0.03;
    public static double trIntensScaling = 0.1;
    public double tagX;

    // turret controller helper
    @SuppressWarnings("SameParameterValue")
    boolean isTriggerPressed(float currentTrigger, float threshold) {
        return currentTrigger > threshold;
    }

    // turret controller
    public void turretController() {
        // check if there is a valid target
        if (!mds.cameraServices.currentDetections.isEmpty()) {
            trDetection = mds.cameraServices.currentDetections.get(0);
            // check if identified tags are one of the targets and have valid pose information
            if (trDetection.metadata != null && trDetection.ftcPose != null && ((!trLoveAtFirstSight && Objects.equals(trDetection.metadata.name, trTarget)) || (trLoveAtFirstSight && (Objects.equals(trDetection.metadata.name, "RedTarget") || Objects.equals(trDetection.metadata.name, "BlueTarget"))))) {
                if (trLoveAtFirstSight) {
                    trTarget = trDetection.metadata.name;
                }
                trTargetFound = true;
            } else {
                trTargetFound = false;
            }
        } else {
            // no tags found
            trTargetFound = false;
        }

        // turret movement control
        if ((trMode.equals("scanLeft") || trMode.equals("scanRight")) && !trTargetFound) {
            // scan for target modes
            if (trMode.equals("scanLeft")) {
                mds.hardware.turretRotation.setPower(0.75);
            }
            if (trMode.equals("scanRight")) {
                mds.hardware.turretRotation.setPower(-0.75);
            }
        } else //noinspection ConstantValue
            if ((trMode.equals("scanLeft") || trMode.equals("scanRight")) && trTargetFound) {
            // stop turret if nothing found and not a scanning mode
            trMode = "seeking";
            mds.hardware.turretRotation.setPower(0);
        }
        if (trTargetFound && trMode.equals("seeking")) {
            // the "seeking" mode, rotates the turret to center on the target
            // determine the horizontal
            tagX = trDetection.ftcPose.x;
            // calculate reduction factor based on proximity to center
            double reductionFactor = 1.0;
            if (Math.abs(tagX) > trSens) {
                double distanceFromSens = Math.abs(tagX) - trSens;
                double maxScaleDistance = trSens * 2; // max distance for full reduction
                // linear reduction
                reductionFactor = Math.max((distanceFromSens / maxScaleDistance) * trIntensScaling, 0.1);
            }
            if (tagX >= trSens) {
                // left
                double scaledTBIintens = trIntens * reductionFactor;
                mds.hardware.turretRotation.setPower(-scaledTBIintens);
            } else if (tagX <= -trSens) {
                // right
                double scaledTBIintens = trIntens * reductionFactor;
                mds.hardware.turretRotation.setPower(scaledTBIintens);
            } else {
                mds.hardware.turretRotation.setPower(0);
            }
        } else if (trMode.equals("seeking")) {
            mds.hardware.turretRotation.setPower(0);
        }

        if (trMode.equals("none")) {
            mds.hardware.turretRotation.setPower(0);
        }

        if (isTriggerPressed(gamepad2.left_trigger, 0.5f)) {
            trMode = "scanLeft";
        } else if (isTriggerPressed(gamepad2.right_trigger, 0.5f)) {
            trMode = "scanRight";
        } else if (gamepad2.right_bumper) {
            trMode = "seeking";
        } else if (gamepad2.left_bumper) {
            trMode = "none";
        }
    }

    // servo controller vars
    public double intakeOn = 0;
    public double outtakeOn = 0;
    public double muOn = 0;
    public long lastMUChangeTime = 0;
     public static double outtakeAngle0 = 0.55;
    public static double outtakeAngle1 = 0.78;

    // lasts (servo controller)
    double intakeOnLast;
    double outtakeOnLast;
    double muOnLast;

    public void servoController() {
        // toggle intake state
        if(!mds.mode.equals("auton")) {
            if (gamepad2.cross && !pgamepad2.cross) {
                intakeOn = (intakeOn == 0) ? 1 : 0;
            }
        }
        // apply intake state if it is different
        if ((intakeOnLast != intakeOn) || firstCycle) {
            if (intakeOn == 1) {
                mds.hardware.intakeLeft.setPosition(1.0);
                mds.hardware.intakeRight.setPosition(-1.0);
                mds.hardware.intakeTop.setPosition(1.0);
            } else {
                mds.hardware.intakeLeft.setPosition(0.5);
                mds.hardware.intakeRight.setPosition(0.5);
                mds.hardware.intakeTop.setPosition(0.5);
            }
        }

        intakeOnLast = intakeOn;

        // determine outtake state
        if(!mds.mode.equals("auton")) {
            if (gamepad2.square) {
                outtakeOn = 1;
            } else {
                outtakeOn = 0;
            }
        }
        // apply outtake state if it is different
        if ((outtakeOnLast != outtakeOn) || firstCycle) {
            if (outtakeOn == 1) {
                mds.hardware.outtake.setPosition(outtakeAngle1);
            } else {
                mds.hardware.outtake.setPosition(outtakeAngle0);
            }
        }

        // determine movUp state
        if(!mds.mode.equals("auton")) {
            if (gamepad2.circle && !pgamepad2.circle) {
                // toggle on/off
                muOn = (muOn == 0) ? 1 : 0;
                lastMUChangeTime = currentTime;
            } else if (gamepad2.dpad_down) {
                // reverse
                muOn = 2;
            } else if (muOn == 2 && !gamepad2.dpad_down) {
                // only do reverse if dpad_down is held
                muOn = 0;
            }
        }
        // apply movUp state if it is different, or if outtake state is different
        if ((muOnLast != muOn || outtakeOn != outtakeOnLast) || firstCycle) {
            if (muOn == 1 && outtakeOn == 0) {
                mds.hardware.movUp.setPower(0.7);
            } else if (muOn == 1 && outtakeOn == 1) {
                if (mds.mode.equals("auton")) {
                    mds.hardware.movUp.setPower(0.95);
                } else {
                    mds.hardware.movUp.setPower(1);
                }
            } else if (muOn == 2) {
                mds.hardware.movUp.setPower(-0.5);
            } else {
                mds.hardware.movUp.setPower(0);
            }
        }

        outtakeOnLast = outtakeOn;
        muOnLast = muOn;
    }

    // angPwrInterpolator
    public HashMap<Double, Double> angPwrMapFunc() {
        HashMap<Double, Double> angPwrMap = new HashMap<>();
        // angPwrMap.put(key, power);
        angPwrMap.put(1.0, 2700.0);
        angPwrMap.put(980.0, 2700.0);
        angPwrMap.put(1220.0, 3000.0);
        angPwrMap.put(1480.0, 3350.0);
        angPwrMap.put(1680.0, 3500.0);
        angPwrMap.put(1920.0, 3600.0);
        angPwrMap.put(2200.0, 4000.0);
        angPwrMap.put(10000.0, 4000.0);

        return angPwrMap;
    }
    public Double angPwrInterpolator(double key) {
        key = key + 150;
        // Sort keys for interpolation
        HashMap<Double, Double> angPwrMap = angPwrMapFunc();
        List<Double> keys = new ArrayList<>(angPwrMap.keySet());
        keys.sort(Double::compare);

        // Check for exact match
        if (angPwrMap.containsKey(key)) {
            return angPwrMap.get(key);
        }

        // Find two surrounding keys for interpolation
        double lowerKey = Double.NEGATIVE_INFINITY;
        double upperKey = Double.POSITIVE_INFINITY;

        for (Double k : keys) {
            if (k < key) {
                lowerKey = k;
            } else if (k > key && upperKey == Double.POSITIVE_INFINITY) {
                upperKey = k;
            }
        }

        Double lowerValue = angPwrMap.get(lowerKey);
        Double upperValue = angPwrMap.get(upperKey);

        if (lowerValue == null || upperValue == null) {
            return 0.0;
        }

        // Linear interpolation
        double t = (key - lowerKey) / (upperKey - lowerKey);

        return lowerValue + t * (upperValue - lowerValue);
    }

    // shooter controller vars
    public static double shooterTPSClose = 2100;
    public static double shooterTPSFar = 2300;
    public int TICKS_PER_REVOLUTION = 28;
    public double flywheelRPM = 0;
    public static double P = 370;
    public static double I = 0.4;
    public static double D = 0;
    public static double F = 6;
    public double shooterPower = 0;
    double shooterPowerLast = shooterPower;
    public double selectedAngle = 0;
    double selectedAngleLast = selectedAngle;
    public double shooterOn = 0;
    double shooterOnLast = shooterOn;
    public double noTagCount = 0;
    public double autoAngPwr = 0;
    public void setPIDFCoefficients(DcMotorEx motor, double P, double I, double D, double F) {
        assert motor != null;
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
    }

    // shooter controller helper functions
    public double getFlywheelRPM() {
        double vel = mds.hardware.flywheel.getVelocity();
        return (vel * 60) / 28;
    }
    public double RPM2TPS(double rpm) {
        int ticksPerRevolution = TICKS_PER_REVOLUTION;
        return (rpm / 60) * ticksPerRevolution;
    }

    public void shooterController() {
        setPIDFCoefficients(mds.hardware.flywheel, P, I, D, F);
        flywheelRPM = getFlywheelRPM();

        // shooter state toggle
        if (gamepad2.triangle && !pgamepad2.triangle) {
            shooterOn = (shooterOn == 0) ? 1 : 0;
        }

        // apply shooter state if it is different
        if (((shooterPowerLast != shooterPower) || (shooterOnLast != shooterOn) || ((shooterPowerLast != shooterPower) && (shooterOnLast != shooterOn))) || firstCycle) {
            if (shooterOn == 1) {
                mds.hardware.flywheel.setVelocity(RPM2TPS(shooterPower));
            } else {
                mds.hardware.flywheel.setVelocity(0);
            }
        }

        shooterPowerLast = shooterPower;
        shooterOnLast = shooterOn;

        // change selected preset angle + power
        if (gamepad2.dpad_up && !pgamepad2.dpad_up) {
            selectedAngle = (selectedAngle == 0) ? 1 : 0;
        }
        if (gamepad2.dpad_right && !pgamepad2.dpad_right) {
            autoAngPwr = (autoAngPwr == 0) ? 1 : 0;
        }
        if (autoAngPwr == 0) {
            // apply selected preset power if it is different
            if ((selectedAngleLast != selectedAngle) || firstCycle) {
                if (selectedAngle == 1) {
                    shooterPower = shooterTPSFar;
                } else {
                    shooterPower = shooterTPSClose;
                }
            }

            selectedAngleLast = selectedAngle;

        } else {
            // interpolator-based speed
            if (mds.cameraServices.currentDetections.isEmpty()) {
                // cant identify any tags
                angPwrFailureFX();
            } else {
                for (AprilTagDetection detection : mds.cameraServices.currentDetections) {
                    if (detection.metadata != null && detection.ftcPose != null && (Objects.equals(detection.metadata.name, "RedTarget") || Objects.equals(detection.metadata.name, "BlueTarget"))) {
                        // determine speed based on range (interpolator)
                        double range = detection.ftcPose.range;
                        shooterPower = angPwrInterpolator(range);
                        noTagCount = 0;
                    } else {
                        // identified tags aren't one of the targets or dont have pose information
                        angPwrFailureFX();
                    }
                }
            }

        }

        // speed reached alert
        if (Math.abs(flywheelRPM - shooterPower) <= 10) {
            tgtReachedFX();
        }

        // copy gamepads to previous vars for rising edge detectors
        pgamepad1.copy(gamepad1);
        pgamepad2.copy(gamepad2);
    }

    // gamepad fx builders
    Gamepad.LedEffect angPwrFailureR = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0.5, 100) // Show purple for 100ms
            .addStep(0, 0, 0, 150) // Pause for for 150ms
            .addStep(1, 0, 0.5, 100) // Show purple for 100ms
            .addStep(0, 0, 0, 150) // Pause for for 150ms
            .addStep(1, 0, 0.5, 100) // Show purple for 100ms
            .addStep(0, 0, 0, 150) // Pause for for 150ms
            .build();
    Gamepad.RumbleEffect angPwrFailureE = new Gamepad.RumbleEffect.Builder()
            .addStep(0.5, 0, 50)  //  Rumble one motor 50% for 50 mSec
            .addStep(0, 0, 137)   //  Pause for 173 mSec
            .addStep(0.0, 1, 50)  //  Rumble other (weaker) motor 100% for 50 mSec
            .addStep(0, 0, 137)
            .addStep(0.5, 0, 50)
            .addStep(0, 0, 137)
            .addStep(0.0, 1, 50)
            .addStep(0, 0, 137)
            .build();
    Gamepad.LedEffect tgtReachedR = new Gamepad.LedEffect.Builder()
            .addStep(0, 1, 0.25, 100) // Show cyan-ish-green for 100ms
            .addStep(0, 0, 0, 150) // Pause for for 150ms
            .build();
    Gamepad.RumbleEffect tgtReachedE = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 100)  //  Rumble left motor 100% for 100 mSec
            .addStep(0, 0.0, 150)  //  Pause for 150 mSec
            .build();

    // play gamepad fx (both FX functions)
    public double lastEffectPlayTime = 0;
    public void angPwrFailureFX() {
        // cant find aprilTag
        if (noTagCount <= 3) {
            if ((currentTime - lastEffectPlayTime) > 750) {
                lastEffectPlayTime = currentTime;
                gamepad1.runLedEffect(angPwrFailureR);
                //gamepad2.runLedEffect(angPwrFailureR);
                gamepad1.runRumbleEffect(angPwrFailureE);
                gamepad2.runRumbleEffect(angPwrFailureE);
            }
        } else {
            noTagCount++;
        }
        currentTime = System.currentTimeMillis();
    }
    public void tgtReachedFX() {
        // target shooter speed reached
        if ((currentTime - lastEffectPlayTime) > 750) {
            lastEffectPlayTime = currentTime;
            gamepad1.runLedEffect(tgtReachedR);
            gamepad2.runLedEffect(tgtReachedR);
            gamepad1.runRumbleEffect(tgtReachedE);
            gamepad2.runRumbleEffect(tgtReachedE);
        }
        currentTime = System.currentTimeMillis();
    }

    // auton helper functions
    public void groupOff() {
        muOn = 0;
        shooterOn = 0;
        intakeOn = 0;
    }
    public void groupOn() {
        muOn = 1;
        shooterOn = 1;
        intakeOn = 1;
    }
    public void autonControllers() {
        servoController();
        shooterController();
        liftController(0);
        turretController();
        firstCycle = false;
        if (!mds.mode.equals("auton")) {
            // copy gamepads to previous vars for rising edge detectors
            // dont do this if mode is auton (lock controllers)
            pgamepad1.copy(gamepad1);
            pgamepad2.copy(gamepad2);
        }
    }
}
