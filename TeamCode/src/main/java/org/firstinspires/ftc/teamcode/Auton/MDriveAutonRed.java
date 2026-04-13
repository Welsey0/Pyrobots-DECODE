// MDrive Auton Red
package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.teamcode.Modules.Auton;
import org.firstinspires.ftc.teamcode.Modules.Drive;

@Autonomous(name="Auton Red")
public class MDriveAutonRed extends OpMode {
    private MDS mds;

    @SuppressWarnings("unused")
    public String auton(String motorcode) {
        switch((int) mds.auton.autonStep) {
            case 0:
                // go to shooting position
                Auton.xtarget = -900;
                Auton.ytarget = 20;
                Auton.moveTime = 6000;
                mds.otherControllers.intakeOn = 1;
                mds.otherControllers.shooterOn = 1;
                mds.otherControllers.outtakeOn = 0;
                return mds.auton.driveStep(1);
            case 1:
                mds.otherControllers.muOn = 1;
                // shoot
                mds.auton.shootCycle(2);
                return mds.auton.holdTargetPosition();
            case 2:
                // determine next step
                if (mds.auton.autonIter != 3) {
                    // if we haven't done this 3 times yet, go to step 3
                    mds.auton.autonStep = 3;
                } else {
                    // if we've done a shoot cycle 3 times, go to parking step
                    mds.otherControllers.groupOff();
                    mds.auton.autonStep = 1000;
                }
                mds.auton.firstOfStep = true;
                return mds.auton.holdTargetPosition();
            case 3:
                // head to intake position
                // set target based on iter
                if (mds.auton.autonIter == 0) {
                    // first row
                    Auton.xtarget = -1300;
                    Auton.ytarget = -300;
                } else if (mds.auton.autonIter == 1) {
                    // second row
                    Auton.xtarget = -1550;
                    Auton.ytarget = -890;
                    Drive.speed = 1.7;
                } else if (mds.auton.autonIter == 2){
                    // third row
                    Auton.xtarget = -1960;
                    Auton.ytarget = -1330;
                    Drive.speed = 1.7;
                }
                Auton.targetHeading = (3.1*3.1415) / 4;
                Auton.distsensitive = 60;
                mds.otherControllers.groupOn();
                mds.otherControllers.outtakeOn = 0;
                return mds.auton.driveStep(4);
            case 4:
                // intake balls
                // set target based on iter
                if (mds.auton.autonIter == 0){
                    Auton.xtarget = -451;
                    Auton.ytarget = -849;
                    Drive.speed = 0.55;
                    Auton.moveTime = 2600;
                } else if (mds.auton.autonIter == 1){
                    Auton.xtarget = -715;
                    Auton.ytarget = -1525;
                    Drive.speed = 0.45;
                    Auton.moveTime = 2600;
                } else if (mds.auton.autonIter == 2){
                    Auton.xtarget = -1050;
                    Auton.ytarget = -2270;
                    Drive.speed = 0.55;
                    Auton.moveTime = 2000;
                }

                mds.otherControllers.groupOn();
                mds.auton.currentTime = System.currentTimeMillis();
                Auton.distsensitive = 110;
                Auton.rotsensitive = 0.2;
                String mc = mds.auton.driveStep(5);
                return mds.auton.targetBallIntake(mc);
            case 5:
                mds.auton.autonStep= 0;
                mds.auton.firstOfStep = true;
                Auton.distsensitive = 50;
                Auton.rotsensitive = 0.1;
                if (mds.auton.autonIter == 0){
                    Auton.targetHeading = 0.15 * (3.1415 / 12);
                }
                if (mds.auton.autonIter == 1){
                    Auton.targetHeading = 0.15 * (3.1415 / 12);
                }
                if (mds.auton.autonIter == 2){
                    Auton.targetHeading = 0.15 * (3.1415 / 12);
                }
                Drive.speed = 1;
                mds.auton.autonIter++;
                return mds.auton.holdTargetPosition();
            case 1000:
                // go to park
                Auton.xtarget = -1000;
                Auton.ytarget = 500;
                mds.otherControllers.groupOff();
                mds.otherControllers.outtakeOn = 0;
                return mds.auton.driveStep(1001);
            case 1001:
                // sit still
                mds.otherControllers.groupOff();
                mds.otherControllers.outtakeOn = 0;
                return mds.auton.holdTargetPosition();
            default:
                // if step invalid, do nothing at all
                mds.otherControllers.groupOff();
                mds.otherControllers.outtakeOn = 0;
                return "0.0,0.0,0.0,0.0";
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