package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MDS;

public class Hardware {
    @SuppressWarnings({"unused", "FieldCanBeLocal"})
    private MDS mds = null;
    public void setMDS(MDS mds) {
        this.mds = mds;
    }
    public HardwareMap hardwareMap;
    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("FL");
        backLeft = hardwareMap.dcMotor.get("BL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backRight = hardwareMap.dcMotor.get("BR");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");

//        intakeLeft = hardwareMap.servo.get("intakeLeft");
//        intakeRight = hardwareMap.servo.get("intakeRight");
//        intakeTop = hardwareMap.servo.get("intakeTop");
//        outtake = hardwareMap.servo.get("outtake");
//        movUp = hardwareMap.dcMotor.get("movUp");
//        turretRotation = hardwareMap.crservo.get("shootangle");
//
//        slideFront = hardwareMap.dcMotor.get("slideFront");
//        slideBack = hardwareMap.dcMotor.get("slideBack");

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            if (pinpoint != null) {
                pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                        org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM,
                        0,
                        0,
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                        0));
            }
        } catch (IllegalArgumentException ignored) {
            pinpoint = null;
        }

        cam1 = hardwareMap.get(WebcamName.class, "webcam1");
//        cam2 = hardwareMap.get(WebcamName.class, "camera2");

        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

//        flywheel.setDirection(DcMotor.Direction.FORWARD);
//        slideFront.setDirection(DcMotor.Direction.FORWARD);
//        slideBack.setDirection(DcMotor.Direction.FORWARD);

//        movUp.setDirection(DcMotor.Direction.REVERSE);
//
//        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        pinpoint.setOffsets(-125.0, 170.0, DistanceUnit.MM);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        IMU.Parameters IMUparameters;
        IMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(IMUparameters);
    }

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public Servo intakeLeft;
    public Servo intakeRight;
    public Servo intakeTop;
    public Servo outtake;
    public DcMotor movUp;
    public DcMotor turretRotation;

    public DcMotorEx shooter;
//    public DcMotor slideFront;
//    public DcMotor slideBack;

    public GoBildaPinpointDriver pinpoint;

    public WebcamName cam1;
//    public WebcamName cam2;

    public IMU imu;
}
