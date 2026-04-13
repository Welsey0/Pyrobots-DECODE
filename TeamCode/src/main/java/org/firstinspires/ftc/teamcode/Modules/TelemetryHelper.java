package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import java.util.Locale;

public class TelemetryHelper {
    private MDS mds = null;
    public void setMDS(MDS mds) {
        this.mds = mds;
    }

    Telemetry dtelemetry;
    public TelemetryHelper(Telemetry itelemetry) {
        this.dtelemetry = itelemetry;
        this.telemetry = new MultipleTelemetry(dtelemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public Telemetry telemetry;

    public void all() {
        core();
        locator();
        camera();
        turret();
        shooter();
        auton();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.update();
    }

    public void core() {
        telemetry.addLine("=== CORE ===");
        telemetry.addData("Mode", mds.mode);
        telemetry.addData("Motor Powers", mds.motorcode);
        mds.devTools.updatefps();
        telemetry.addLine("--- Performance ---");
        telemetry.addData("Raw FPS", mds.devTools.fps);
        telemetry.addData("Average FPS", mds.devTools.average);
        telemetry.addData("1% Lows", mds.devTools.low);
        telemetry.addData("Frametime", mds.devTools.frametime);
        telemetry.addLine("------");
        telemetry.addLine("======\n");
    }

    public void locator() {
        telemetry.addLine("=== LOCATOR ===");
        telemetry.addData("Location", mds.locator.odoLoc);
        telemetry.addData("Pinpoint Status", mds.locator.odoStat);
        telemetry.addLine("======\n");
    }

    public void camera() {
        telemetry.addLine("=== CAMERA ===");
        if (CameraServices.camMode.equals("tag")) {
            for (AprilTagDetection detection : mds.cameraServices.currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format(Locale.US, "--- Detection: %s (ID %d) ---", detection.metadata.name, detection.id));
                    telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (mm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (mm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format(Locale.US, "--- Unknown Detection (ID %d) ---", detection.id));
                    telemetry.addLine(String.format(Locale.US, "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
            if (!mds.cameraServices.currentDetections.isEmpty()) {
                telemetry.addLine("--- Key ---");
                telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
                telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                telemetry.addLine("RBE = Range, Bearing & Elevation");
                telemetry.addLine("------");
            } else {
                telemetry.addLine("---\nNo Detections\n---");
            }
        } else if (CameraServices.camMode.equals("ball")) {
            for (ColorBlobLocatorProcessor.Blob b : mds.cameraServices.currentBlobs) {
                Circle circleFit = b.getCircle();
                telemetry.addLine("--- Ball Detection ---");
                telemetry.addLine(String.format(Locale.US, "CR(XY) %5.3f %3d (%3d,%3d)", b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
                telemetry.addLine("------");
            }
            if (!mds.cameraServices.currentBlobs.isEmpty()) {
                telemetry.addLine("--- Key ---");
                telemetry.addLine("CR(XY) = Circularity, Radius, Circle Fit X, Circle Fit Y");
                telemetry.addLine("------");
            } else {
                telemetry.addLine("---\nNo Detections\n---");
            }
        }
        telemetry.addLine("======\n");
    }

    public void turret() {
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("LAFS", OtherControllers.trLoveAtFirstSight);
        telemetry.addData("Mode", mds.otherControllers.trMode);
        telemetry.addData("Target", OtherControllers.trTarget);
        telemetry.addData("Distance to Target", mds.otherControllers.tagX);
        telemetry.addLine("======\n");
    }

    public void shooter() {
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Shooter RPM", mds.otherControllers.flywheelRPM);
        telemetry.addData("Target RPM", mds.otherControllers.shooterPower);
        telemetry.addData("Shooter On", mds.otherControllers.shooterOn);
        telemetry.addData("Interpolator On", mds.otherControllers.autoAngPwr);
        telemetry.addLine("======\n");
    }

    public void auton() {
        telemetry.addLine("=== AUTON ===");
        telemetry.addData("Step", mds.auton.autonStep);
        telemetry.addData("Current X", mds.auton.curX);
        telemetry.addData("Target X", Auton.xtarget);
        telemetry.addData("Current Y", mds.auton.curY);
        telemetry.addData("Target y", Auton.ytarget);
        telemetry.addData("Current Heading", mds.auton.curHeading);
        telemetry.addData("Target Heading", Auton.targetHeading);
        telemetry.addData("Distance to Target", mds.auton.distanceToTarget);
        telemetry.addLine("======\n");
    }
}