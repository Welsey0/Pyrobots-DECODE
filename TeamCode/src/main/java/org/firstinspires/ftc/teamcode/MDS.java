package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Modules.Auton;
import org.firstinspires.ftc.teamcode.Modules.CameraServices;
import org.firstinspires.ftc.teamcode.Modules.DevTools;
import org.firstinspires.ftc.teamcode.Modules.Drive;
import org.firstinspires.ftc.teamcode.Modules.Hardware;
import org.firstinspires.ftc.teamcode.Modules.Locator;
import org.firstinspires.ftc.teamcode.Modules.OtherControllers;
import org.firstinspires.ftc.teamcode.Modules.TelemetryHelper;

// Modular Drive System
// Core + Module Initialization

public class MDS {
    public HardwareMap hardwareMap;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public MultipleTelemetry telemetry;

    public MDS(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, MultipleTelemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        // instantiate modules now that basic inputs are assigned
        this.auton = new Auton();
        this.cameraServices = new CameraServices();
        this.devTools = new DevTools();
        this.drive = new Drive(this.gamepad1, this.gamepad2);
        this.hardware = new Hardware(this.hardwareMap);
        this.locator = new Locator();
        this.otherControllers = new OtherControllers(this.gamepad1, this.gamepad2);
        this.telemetryHelper = new TelemetryHelper(this.telemetry);
    }

    public Auton auton;
    public CameraServices cameraServices;
    public DevTools devTools;
    public Drive drive;
    public Hardware hardware;
    public Locator locator;
    public OtherControllers otherControllers;
    public TelemetryHelper telemetryHelper;

    public String mode = "unset";
    public String motorcode = "0.0,0.0,0.0,0.0";

    public void init(MDS mds, String omode) {
        mode = omode;

        auton.setMDS(mds);
        cameraServices.setMDS(mds);
        devTools.setMDS(mds);
        drive.setMDS(mds);
        hardware.setMDS(mds);
        locator.setMDS(mds);
        otherControllers.setMDS(mds);
        telemetryHelper.setMDS(mds);

        hardware.init();
        hardware.pinpoint.resetPosAndIMU();
        auton.init();
        cameraServices.init();
        otherControllers.init();
    }
}