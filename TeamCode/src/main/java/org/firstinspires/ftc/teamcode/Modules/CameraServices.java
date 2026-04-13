package org.firstinspires.ftc.teamcode.Modules;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@Config
public class CameraServices {
    private MDS mds = null;
    public void setMDS(MDS mds) {
        this.mds = mds;
    }
    public static Position cam1Position = new Position(DistanceUnit.MM, 225, 120, 0, 0);
    public static YawPitchRollAngles cam1Orientation = new YawPitchRollAngles(AngleUnit.DEGREES, 7, 0, 0, 0);

    public static boolean dashCamStream = false;
    public VisionPortal visionPortal;
    public VisionPortal visionPortal2;
    public static AprilTagProcessor aprilTag;
    public static ColorBlobLocatorProcessor ballDetectG;
    public static ColorBlobLocatorProcessor ballDetectP;
    public static String camMode;

    public void init() {
        if (mds.mode.equals("teleop")) {
            camMode = "tag";
        } else if (mds.mode.equals("auton")) {
            camMode = "ball";
        }
        if (camMode.equals("tag")) {
            aprilTag = new AprilTagProcessor.Builder()
                    .setCameraPose(cam1Position, cam1Orientation)
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setDrawTagID(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                    .build();

            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.addProcessor(aprilTag);
            builder.enableLiveView(false);
            builder.setCamera(mds.hardware.cam1);
            visionPortal = builder.build();
        } else if (camMode.equals("ball")) {
            ballDetectG = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                    .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setBoxFitColor(0)
                    .setCircleFitColor(Color.rgb(220, 0, 255))
                    .setBlurSize(5)
                    .setDilateSize(15)
                    .setErodeSize(15)
                    .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                    .build();

            ballDetectP = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setBoxFitColor(0)
                    .setCircleFitColor(Color.rgb(0, 255, 55))
                    .setBlurSize(5)
                    .setDilateSize(15)
                    .setErodeSize(15)
                    .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                    .build();

            VisionPortal.Builder builder2 = new VisionPortal.Builder();
            builder2.addProcessors(ballDetectG, ballDetectP);
            builder2.enableLiveView(false);
            builder2.setCamera(mds.hardware.cam2);
            visionPortal2 = builder2.build();
        }

        if (dashCamStream) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            if (Objects.equals(camMode, "tag")) {
                dashboard.startCameraStream(visionPortal, 0);
            } else if (Objects.equals(camMode, "ball")) {
                dashboard.startCameraStream(visionPortal2, 0);
            }
        }
    }

    public List<AprilTagDetection> currentDetections = new ArrayList<>();

    public List<ColorBlobLocatorProcessor.Blob> currentBlobs = new ArrayList<>();

    public void update_april() {
        currentDetections = aprilTag.getDetections();
    }

    public void update_blobs() {
        currentBlobs.clear();
        currentBlobs.addAll(ballDetectG.getBlobs());
        currentBlobs.addAll(ballDetectP.getBlobs());
    }
}
