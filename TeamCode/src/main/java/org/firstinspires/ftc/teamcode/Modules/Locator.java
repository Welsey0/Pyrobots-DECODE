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
    public String odoLoc = "";
    public Pose2D odoPos = null;
    public String odoStat = "";
    public double curHeading;
    public void update_odometry() {
        mds.hardware.pinpoint.update();
        Pose2D pos = mds.hardware.pinpoint.getPosition();
        odoStat = String.valueOf(mds.hardware.pinpoint.getDeviceStatus());
        if (pos != null) {
            odoPos = pos;
            curHeading = odoPos.getHeading(AngleUnit.RADIANS);
            if (Double.isNaN(curHeading)) {
                YawPitchRollAngles robotOrientation;
                robotOrientation = mds.hardware.imu.getRobotYawPitchRollAngles();
                curHeading = robotOrientation.getYaw(AngleUnit.DEGREES);
            }
            odoLoc = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), curHeading);
        } else {
            odoLoc = "Unknown";
        }
    }
}
