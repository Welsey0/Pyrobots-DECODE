package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MDS;

import java.util.Locale;

public class Locator {
    private MDS mds;
    public void setMDS(MDS mds) {
        this.mds = mds;
    }

    public String odoLoc = "Unknown";
    public Pose2D odoPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    public String odoStat = "No pinpoint";
    public double curHeading = 0.0;
    public boolean usingPinpoint = false;

    public void init() {
        if (mds == null || mds.hardware == null) {
            return;
        }

        if (mds.hardware.pinpoint != null) {
            usingPinpoint = true;
            mds.hardware.pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));
            odoPos = mds.hardware.pinpoint.getPosition();
            if (odoPos != null) {
                curHeading = odoPos.getHeading(AngleUnit.RADIANS);
            }
        } else {
            usingPinpoint = false;
            odoPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
            curHeading = 0.0;
        }
    }

    public void update_odometry() {
        if (mds == null || mds.hardware == null) {
            odoLoc = "No hardware";
            return;
        }

        if (mds.hardware.pinpoint != null) {
            usingPinpoint = true;
            mds.hardware.pinpoint.update();
            Pose2D pos = mds.hardware.pinpoint.getPosition();
            if (pos != null) {
                odoPos = pos;
                curHeading = odoPos.getHeading(AngleUnit.RADIANS);
                odoStat = String.valueOf(mds.hardware.pinpoint.getDeviceStatus());
                odoLoc = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f",
                        odoPos.getX(DistanceUnit.MM),
                        odoPos.getY(DistanceUnit.MM),
                        curHeading);
                return;
            }
            odoStat = "No position";
        } else {
            usingPinpoint = false;
            if (mds.hardware.imu != null) {
                YawPitchRollAngles robotOrientation = mds.hardware.imu.getRobotYawPitchRollAngles();
                curHeading = Math.toRadians(robotOrientation.getYaw(AngleUnit.DEGREES));
                odoPos = new Pose2D(DistanceUnit.MM,
                        odoPos.getX(DistanceUnit.MM),
                        odoPos.getY(DistanceUnit.MM),
                        AngleUnit.RADIANS,
                        curHeading);
                odoStat = "IMU fallback";
                odoLoc = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f",
                        odoPos.getX(DistanceUnit.MM),
                        odoPos.getY(DistanceUnit.MM),
                        curHeading);
                return;
            }
        }

        odoLoc = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f",
                odoPos.getX(DistanceUnit.MM),
                odoPos.getY(DistanceUnit.MM),
                curHeading);
    }
}
