// MDrive Auton Red [Legacy, Do Not Run]
package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.teamcode.Modules.Auton;
import org.firstinspires.ftc.teamcode.Modules.Drive;
import org.firstinspires.ftc.teamcode.Modules.OtherControllers;

@Disabled
@Autonomous(name="Auton Red [Legacy, Do Not Run]")
public class MDriveAutonRedLegacy extends OpMode {
    private MDS mds;

    private String auton(String motorcode) {
        if (mds.auton.autonStep == 0) {
            if (mds.auton.firstOfStep) {
                Auton.xtarget = -1000;
                Auton.ytarget = 20;
                mds.auton.firstOfStep = false;
                mds.auton.autonMoving = 1;
            }
            mds.otherControllers.muOn = 1;
            mds.otherControllers.shooterOn = 1;
            mds.otherControllers.intakeOn = 1;
            mds.hardware.outtake.setPosition(OtherControllers.outtakeAngle0);
            mds.otherControllers.autonControllers();
            motorcode = mds.auton.driveToPosition();
            if (mds.auton.autonMoving == 0) {
                mds.auton.autonStep = 1;
                mds.auton.firstOfStep = true;
            }
            return motorcode;
        }else if (mds.auton.autonStep == 1) {
            if (mds.auton.firstOfStep) {
                mds.auton.autonShotsFired = 0;
                Auton.xtarget = -1000;
                Auton.ytarget = 20;
                mds.auton.firstOfStep = false;
                motorcode = mds.auton.driveToPosition();
            }
            if (mds.auton.autonIter == 0){
                mds.auton.justWait(1000);
            }
            mds.auton.justWait(100);
            mds.hardware.outtake.setPosition(OtherControllers.outtakeAngle1);
            mds.auton.justWait(300);
            mds.hardware.outtake.setPosition(OtherControllers.outtakeAngle0);
            mds.auton.justWait(1200);
            mds.hardware.outtake.setPosition(OtherControllers.outtakeAngle1);
            mds.auton.justWait(300);
            mds.hardware.outtake.setPosition(OtherControllers.outtakeAngle0);
            mds.auton.justWait(1200);
            mds.hardware.outtake.setPosition(OtherControllers.outtakeAngle1);
            mds.auton.justWait(300);
            mds.hardware.outtake.setPosition(OtherControllers.outtakeAngle0);

            if (mds.auton.autonIter != 2) {
                mds.auton.autonStep = 2;
                mds.otherControllers.servoController();
                mds.otherControllers.shooterController();
                mds.auton.firstOfStep = true;
            }else{
                mds.otherControllers.shooterOn = 0;
                mds.otherControllers.muOn = 0;
                mds.otherControllers.intakeOn = 0;
                mds.auton.firstOfStep = true;
                mds.otherControllers.servoController();
                mds.otherControllers.shooterController();
                mds.auton.autonStep = 1000;
            }
            return motorcode;
        } else if (mds.auton.autonStep == 2) {
            //skip this step and reset control vars
            mds.auton.firstOfStep = true;
            mds.auton.autonMoving = 1;
            mds.auton.autonStep = 3;
            mds.otherControllers.autonControllers();
            return "0.00,0.00,0.00,0.00";
        } else if (mds.auton.autonStep == 3) {
            //head to row of balls
            if (mds.auton.firstOfStep) {
                //set targets depending on iter
                if (mds.auton.autonIter == 0 || mds.auton.autonIter == 1) {
                    Auton.targetHeading = (3.1*3.1415) / 4; // red
                    mds.auton.firstOfStep = false;
                    mds.auton.autonMoving = 1;
                }
            }
            motorcode = mds.auton.driveToPosition();
            if (mds.auton.autonMoving == 0) {
                mds.auton.autonStep = 4;
                mds.auton.firstOfStep = true;
            }
            return motorcode;
        } else if (mds.auton.autonStep == 4) {
            //head to row of balls
            if (mds.auton.firstOfStep) {
                //set targets depending on iter
                if (mds.auton.autonIter == 0) {
                    Auton.xtarget = -1300;
                    Auton.ytarget = -300;
                    mds.auton.firstOfStep = false;
                    mds.auton.autonMoving = 1;
                    Auton.distsensitive = 60;
                } else if (mds.auton.autonIter == 1){
                    Auton.xtarget = -1475;
                    Auton.ytarget = -975;
                    mds.auton.firstOfStep = false;
                    mds.auton.autonMoving = 1;
                    Auton.distsensitive = 60;
                }
            }
            motorcode = mds.auton.driveToPosition();
            if (mds.auton.autonMoving == 0) {
                mds.auton.autonStep = 5;
                mds.auton.firstOfStep = true;
            }
            return motorcode;
        }else if (mds.auton.autonStep == 5) {
            //intake balls
            if (mds.auton.firstOfStep) {
                if (mds.auton.autonIter == 0){
                    Auton.xtarget = -451;
                    Auton.ytarget = -849;
                    Drive.speed = 0.55;
                    mds.auton.firstOfStep = false;
                    mds.auton.autonMoving = 1;
                } else if (mds.auton.autonIter == 1){
                    Auton.xtarget = -715;
                    Auton.ytarget = -1525;
                    Drive.speed = 0.45;
                    mds.auton.firstOfStep = false;
                    mds.auton.autonMoving = 1;
                }
                mds.auton.currentTime = System.currentTimeMillis();
                mds.auton.startTime = mds.auton.currentTime;
                Auton.moveTime = 2500; //expected time in millis
            }
            mds.otherControllers.muOn = 1;
            mds.otherControllers.intakeOn = 1;
            Auton.distsensitive = 110;
            Auton.rotsensitive = 0.2;
            mds.auton.currentTime = System.currentTimeMillis();
            mds.otherControllers.autonControllers();
            motorcode = mds.auton.driveToPosition();
            if (mds.auton.autonMoving == 0 || (mds.auton.currentTime - mds.auton.startTime) > Auton.moveTime) {
                Drive.speed = 1;
                //return to first step and increase iter
                mds.auton.autonStep = 0;
                mds.auton.firstOfStep = true;
                mds.auton.autonIter++;
                Auton.distsensitive = 50;
                Auton.rotsensitive = 0.1;
                Auton.targetHeading = 0.2 * (3.1415 / 12);
            }
            return motorcode;
        } else if (mds.auton.autonStep == 1000) {
            //park
            if (mds.auton.firstOfStep) {
                Auton.ytarget = 500;
                mds.auton.firstOfStep = false;
                mds.auton.autonMoving = 1;
            }
            mds.otherControllers.muOn = 0;
            mds.otherControllers.shooterOn = 0;
            mds.otherControllers.outtakeOn = 0;
            mds.otherControllers.intakeOn = 0;
            mds.otherControllers.autonControllers();
            motorcode = mds.auton.driveToPosition();
            if (mds.auton.autonMoving == 0) {
                mds.auton.autonStep = 1001;
            }
            return motorcode;
        } else if (mds.auton.autonStep == 1001) {
            return "0.00,0.00,0.00,0.00";
        } else {
            mds.otherControllers.muOn = 0;
            mds.otherControllers.shooterOn = 0;
            mds.otherControllers.outtakeOn = 0;
            mds.otherControllers.intakeOn = 0;
            mds.otherControllers.autonControllers();
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