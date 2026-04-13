// MDrive
package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.teamcode.Modules.Drive;

@TeleOp(name="MDrive")
public class MDrive extends OpMode {
    private MDS mds;
    @Override
    public void init() {
        MultipleTelemetry mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mds = new MDS(hardwareMap, gamepad1, gamepad2, mtelemetry);
        mds.init(mds,"teleop");
    }

    @Override
    public void loop() {
        if (mds.mode.equals("teleop")) {
            // speed control
            if (gamepad1.left_bumper) {
                Drive.speed = 0.4;
            } else {
                Drive.speed = 1.0;
            }

            // info updates
            mds.cameraServices.update_april();
            mds.locator.update_odometry();

            // direct drive (with autorotate available)
            mds.motorcode = mds.drive.driveFromJoy(1);
            if (gamepad1.right_bumper) {
                mds.motorcode = mds.auton.rotateToTarget(mds.motorcode);
            }
            mds.drive.setPowers(mds.motorcode);

            // relative drive options
            if(gamepad1.share){
                mds.hardware.pinpoint.setHeading(0,AngleUnit.RADIANS);
                mds.hardware.imu.resetYaw();
            }
            if(gamepad1.options){
                mds.hardware.pinpoint.recalibrateIMU();
            }

            // other movement controllers
            mds.otherControllers.liftController(-gamepad2.left_stick_y);
            mds.otherControllers.servoController();
            mds.otherControllers.shooterController();
            mds.otherControllers.turretController();
            mds.otherControllers.firstCycle = false;

            // copy gamepads to previous vars for rising edge detectors
            mds.otherControllers.pgamepad1.copy(gamepad1);
            mds.otherControllers.pgamepad2.copy(gamepad2);
        } else {
            mds.mode = "teleop";
        }
        mds.telemetryHelper.all();
    }
}