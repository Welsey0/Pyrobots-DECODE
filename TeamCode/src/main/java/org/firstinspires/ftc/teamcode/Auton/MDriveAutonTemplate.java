// MDrive Auton Template
package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.teamcode.Modules.Auton;

@Disabled
@Autonomous(name="Auton Template")
public class MDriveAutonTemplate extends OpMode {
    private MDS mds;

    @SuppressWarnings("unused")
    public String auton(String motorcode) {
        switch((int) mds.auton.autonStep) {
            case 0:
                Auton.xtarget = 200;
                Auton.ytarget = 200;
                return mds.auton.driveStep(1);
            case 1:
                Auton.xtarget = 0;
                Auton.ytarget = 0;
                return mds.auton.driveStep(2);
            default:
                mds.otherControllers.groupOff();
                return "0.00,0.00,0.00,0.00";
        }
    }

    @Override
    public void init() {
        MultipleTelemetry mtelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        mds = new MDS(hardwareMap, gamepad1, gamepad2, mtelemetry);
        mds.init(mds,"auton");
    }

    @Override
    public void loop() {
        if (mds.mode.equals("auton")) {
            mds.locator.update_odometry();
            mds.motorcode = auton(mds.motorcode);
            mds.otherControllers.autonControllers();
            mds.drive.setPowers(mds.motorcode);
        } else {
            mds.mode = "auton";
        }
        mds.telemetryHelper.all();
    }
}