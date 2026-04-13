package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.MDS;
import java.util.Locale;
import java.util.Objects;

public class Drive {
    @SuppressWarnings("FieldMayBeFinal")
    private MDS mds = null;
    public void setMDS(MDS mds) {
        this.mds = mds;
    }
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public Drive(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    // Returns a string with four motor powers based on joystick input
    public String driveFromJoy(int controllerNum) {
        Gamepad pad = (controllerNum == 2) ? gamepad2 : gamepad1;
        double dy = -pad.left_stick_y;
        double dx = pad.left_stick_x;
        double rx = -pad.right_stick_x;
        // Transform delta into robot frame (rotate by -curHeading)
        double coos = Math.cos(mds.locator.curHeading);
        double siin = Math.sin(mds.locator.curHeading);
        double x = dx * coos + dy * siin;     // forward (+) in robot frame.
        double y = dy * coos - dx * siin;    // left (+) in robot frame
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double zero = (y + x + rx) / denominator;
        double one = (y - x - rx) / denominator;
        double two = (y - x + rx) / denominator;
        double three = (y + x - rx) / denominator;
        return String.format(Locale.US,"%.2f,%.2f,%.2f,%.2f", zero, one, two, three);
    }
    public static double speed = 1;
    String motorcodeLast;
    // Sets motor powers from motorcode string
    public void setPowers(String motorcode) {
        if (!Objects.equals(motorcode, motorcodeLast)) {
            String[] parts = motorcode.split(",");
            double zero = Double.parseDouble(parts[0]);
            double one = Double.parseDouble(parts[1]);
            double two = Double.parseDouble(parts[2]);
            double three = Double.parseDouble(parts[3]);
            mds.hardware.frontLeft.setPower(zero * speed);
            mds.hardware.backLeft.setPower(two * speed);
            mds.hardware.frontRight.setPower(one * speed);
            mds.hardware.backRight.setPower(three * speed);
            motorcodeLast = motorcode;
        }
    }
}
