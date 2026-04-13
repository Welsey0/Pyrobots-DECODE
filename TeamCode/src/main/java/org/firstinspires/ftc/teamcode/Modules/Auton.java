package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.Locale;

@Config
public class Auton {
    private MDS mds = null;
    public void setMDS(MDS mds) {
        this.mds = mds;
    }

    public void init() {
        controllerx = new PIDController(Kpx, Kix, Kdx);
        controllery = new PIDController(Kpy, Kiy, Kdy);
        controllerrot = new PIDController(Kpr, Kir, Kdr);
    }
    public void justWait(int milliseconds){
        double currTime = System.currentTimeMillis();
        double waitUntil = currTime + (double)(milliseconds);
        //noinspection StatementWithEmptyBody
        while (System.currentTimeMillis() < waitUntil){
        }
    }
    public double autonStep = 0;
    public boolean firstOfStep = true;
    public double autonShotsFired = 0;
    public double autonMoving = 0;
    public double autonShooting = 0;
    public double autonIter = 0;
    public long currentTime = System.currentTimeMillis();
    public long startTime = currentTime;
    public static long moveTime = 3000; //expected time in millis
    public PIDController controllerx;
    public PIDController controllery;
    public PIDController controllerrot;
    public static double Kpx = 0.0023;
    public static double Kix = 0.042;
    public static double Kdx = 0.0005;
    public static double Kpy = 0.0028;
    public static double Kiy = 0.04;
    public static double Kdy = 0.00022;
    public static double Kpr = 0.73;
    public static double Kir = 0.005;
    public static double Kdr = 0.004;
    public static double xtarget = 0;
    public static double ytarget = 0;
    public static double targetHeading = 0;
    public double curX = 0;
    public double curY = 0;
    public double curHeading = 0;
    public double hypotToTarget;
    public static double distsensitive = 50;
    public static double rotsensitive = 0.1;
    public double rotToTarget;
    public String distanceToTarget;

    public String driveToPosition() {

        curX = mds.locator.odoPos.getX(DistanceUnit.MM);
        curY = mds.locator.odoPos.getY(DistanceUnit.MM);
        curHeading = mds.locator.odoPos.getHeading(AngleUnit.RADIANS);

        controllerx.setPID(Kpx, Kix, Kdx);
        controllery.setPID(Kpy, Kiy, Kdy);
        controllerrot.setPID(Kpr, Kir, Kdr);
        double dx = controllerx.calculate(curX, xtarget);
        double dy = controllery.calculate(curY, ytarget);
        double rx = controllerrot.calculate(curHeading, targetHeading);

        // Transform delta into robot frame (rotate by -curHeading)
        double coos = Math.cos(curHeading);
        double siin = Math.sin(curHeading);
        double x = dx * coos + dy * siin;     // forward (+) in robot frame.
        double y_r = dy * coos - dx * siin;    // left (+) in robot frame

        distanceToTarget = String.format(Locale.US, "x = %.2f, y = %.2f, hypot = %.2f", dx, dy, Math.hypot(dx, dy));
        hypotToTarget = Math.hypot(xtarget-curX, ytarget-curY);
        rotToTarget = Math.abs(targetHeading-curHeading);

        if (hypotToTarget <= distsensitive && rotToTarget <= rotsensitive) {
            autonMoving = 0;
            return "0.0,0.0,0.0,0.0";
        }

        // Mecanum math brick
        double y = -y_r * 1.1;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double zero = (x + y + rx) / denominator;
        double one = (x - y - rx) / denominator;
        double two = (x - y + rx) / denominator;
        double three = (x + y - rx) / denominator;

        return String.format(Locale.US, "%.2f,%.2f,%.2f,%.2f", zero, one, two, three);
    }

    public String driveStep(int nextStep) {
        String motorcode;
        currentTime = System.currentTimeMillis();
        if (firstOfStep) {
            firstOfStep = false;
            autonMoving = 1;
            startTime = currentTime;
        }
        motorcode = driveToPosition();
        if (autonMoving == 0 || (currentTime - startTime) > Auton.moveTime) {
            autonStep = nextStep;
            firstOfStep = true;
        }
        return motorcode;
    }
    public String holdTargetPosition() {
        String motorcode;
        autonMoving = 1;
        motorcode = driveToPosition();
        if (autonMoving == 0) {
            return "0.0,0.0,0.0,0.0";
        }
        return motorcode;
    }
    ArrayList<Integer> shootTimings = new ArrayList<>();
    ArrayList<Integer> shootStates = new ArrayList<>();
    long shootStartTime = currentTime;
    int shootIndex = 0;
    public void shootCycle(int nextStep) {
        if (firstOfStep) {
            shootTimings.clear();
            shootStates.clear();
            shootIndex = 0;
            currentTime = System.currentTimeMillis();
            shootStartTime = currentTime;
            firstOfStep = false;
            autonShooting = 1;
            if (autonIter == 0){
                shootTimings.add(1100);
                shootStates.add(0);
            }
            shootTimings.add(100);
            shootStates.add(1);
            shootTimings.add(2000);
            shootStates.add(0);
            // timings contains time in ms for that shoot step, states determines the shooter's power after the time has passed
        }
        currentTime = System.currentTimeMillis();
        mds.otherControllers.groupOn();
        if (shootIndex >= shootTimings.size()) {
            autonShooting = 0;
        } else if ((currentTime - shootStartTime) >= shootTimings.get(shootIndex)) {
            mds.otherControllers.outtakeOn = shootStates.get(shootIndex);
            shootIndex++;
            shootStartTime = currentTime;
        }
        if (autonShooting == 0) {
            autonStep = nextStep;
            firstOfStep = true;
        }
    }

    public String rotateToTarget(String motorcode) {
        if (!mds.cameraServices.currentDetections.isEmpty()) {
            // get target information
            AprilTagDetection goal = mds.cameraServices.currentDetections.get(0);
            double goalBearing = goal.ftcPose.bearing;
            // get current position (to stay there)
            xtarget = mds.locator.odoPos.getX(DistanceUnit.MM);
            ytarget = mds.locator.odoPos.getY(DistanceUnit.MM);
            // determine target heading
            double goalBearingRadians = Math.toRadians(goalBearing);
            double deltaHeading = mds.locator.curHeading + goalBearingRadians;
            //deltaHeading = (deltaHeading + Math.PI) % (2 * Math.PI) - Math.PI;
            rotsensitive = 0.005;
            targetHeading = deltaHeading;
            // get motorcode to turn to that position
            autonMoving = 1;
            String rotMotorcode = driveToPosition();
            // combine motorcodes for final output
            return combineMotorcodes(motorcode, rotMotorcode);
        } else {
            return motorcode;
        }
    }
    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    ColorBlobLocatorProcessor.Blob blob;
    public static double TBIsens = 50;
    public static double TBIintens = 7;
    public static double TBIintensScaling = 150;
    public String targetBallIntake(String motorcode) {
        if (!mds.cameraServices.currentBlobs.isEmpty()) {
            blob = null;
            // determine the biggest in view (closest) ball
            double maxRadius = 0;
            for (ColorBlobLocatorProcessor.Blob b : mds.cameraServices.currentBlobs) {
                Circle circleFit = b.getCircle();
                double radius = circleFit.getRadius();
                if (radius > maxRadius && radius >= 70) {
                    maxRadius = radius;
                    blob = b;
                }
            }
            if (blob == null) {
                return motorcode;
            }
            // determine the horizontal
            double blobX = blob.getCircle().getX() - 270;
            // calculate reduction factor based on proximity to center
            double reductionFactor = 1.0;
            if (Math.abs(blobX) > TBIsens) {
                double distanceFromSens = Math.abs(blobX) - TBIsens;
                double maxScaleDistance = TBIsens * 2; // max distance for full reduction
                // linear reduction
                reductionFactor = Math.max(1.0 - (distanceFromSens / maxScaleDistance) * TBIintensScaling, 0.1);
            }
            if (blobX >= TBIsens) {
                // left
                double scaledTBIintens = TBIintens * reductionFactor;
                String smc = String.format(Locale.US, "%.2f,%.2f,%.2f,%.2f",
                        -scaledTBIintens, scaledTBIintens, scaledTBIintens, -scaledTBIintens);
                return combineMotorcodes(smc, motorcode);
            } else if (blobX <= -TBIsens) {
                // right
                double scaledTBIintens = TBIintens * reductionFactor;
                String smc = String.format(Locale.US, "%.2f,%.2f,%.2f,%.2f",
                        scaledTBIintens, -scaledTBIintens, -scaledTBIintens, scaledTBIintens);
                return combineMotorcodes(motorcode, smc);
            } else {
                return motorcode;
            }
        } else {
            return motorcode;
        }
    }

    private String combineMotorcodes(String mc1, String mc2) {
        String[] parts1 = mc1.split(",");
        String[] parts2 = mc2.split(",");
        if (parts1.length != parts2.length) {
            return mc1;
        }
        double[] blendedParts = new double[parts1.length];
        for (int i = 0; i < parts1.length; i++) {
            double pwr1 = Double.parseDouble(parts1[i]);
            double pwr2 = Double.parseDouble(parts2[i]);
            blendedParts[i] = clamp(pwr1 + pwr2, -1, 1);
        }
        return String.format(Locale.US, "%.2f,%.2f,%.2f,%.2f",
                blendedParts[0], blendedParts[1], blendedParts[2], blendedParts[3]);
    }
}
