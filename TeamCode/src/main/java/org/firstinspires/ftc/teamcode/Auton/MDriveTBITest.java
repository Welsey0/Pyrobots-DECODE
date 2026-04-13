// MDrive TBI Tester
package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.MDS;

@Autonomous(name="Auton TBI Test")
public class MDriveTBITest extends OpMode {
    private MDS mds;

    @SuppressWarnings("unused")
    public String auton(String motorcode) {
        //noinspection SwitchStatementWithTooFewBranches
        switch((int) mds.auton.autonStep) {
            case 0:
                String mc = "0.0,0.0,0.0,0.0";
                return mds.auton.targetBallIntake(mc);
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
            mds.cameraServices.update_blobs();
            mds.motorcode = auton(mds.motorcode);
            mds.otherControllers.autonControllers();
            mds.drive.setPowers(mds.motorcode);
        } else {
            mds.mode = "auton";
        }
        mds.telemetryHelper.all();
    }
}