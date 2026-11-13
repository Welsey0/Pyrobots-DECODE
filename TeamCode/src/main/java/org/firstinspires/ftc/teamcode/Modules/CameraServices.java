package org.firstinspires.ftc.teamcode.Modules;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MDS;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

@Config
public class CameraServices {
    private static final String MODE_TAG = "tag";
    private static final String MODE_BALL = "ball";
    public static final String RED = "red";
    public static final String BLUE = "blue";
    public static String targetAlliance = RED;

    public static int[] redTagBatch = {1, 2, 3, 4};
    public static int[] blueTagBatch = {5, 6, 7, 8};

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
    public static String camMode = MODE_TAG;

    public static void toggleTargetAlliance() {
        targetAlliance = Objects.equals(targetAlliance, RED) ? BLUE : RED;
    }

    public static boolean isTagInAlliance(AprilTagDetection detection, String alliance) {
        if (detection == null) {
            return false;
        }

        String target = alliance == null ? targetAlliance : alliance;
        String normalizedTarget = target == null ? RED : target.toLowerCase(Locale.US);

        String tagName = detection.metadata != null ? detection.metadata.name : "";
        if (tagName != null && !tagName.isEmpty()) {
            String normalizedName = tagName.toLowerCase(Locale.US);
            boolean nameIsRed = normalizedName.contains("red");
            boolean nameIsBlue = normalizedName.contains("blue");
            if (nameIsRed || nameIsBlue) {
                return (normalizedTarget.equals(RED) && nameIsRed) || (normalizedTarget.equals(BLUE) && nameIsBlue);
            }
        }

        int tagId = detection.id;
        for (int id : redTagBatch) {
            if (tagId == id) {
                return normalizedTarget.equals(RED);
            }
        }
        for (int id : blueTagBatch) {
            if (tagId == id) {
                return normalizedTarget.equals(BLUE);
            }
        }

        return false;
    }

    public static List<AprilTagDetection> getTargetDetections(List<AprilTagDetection> detections, String alliance) {
        List<AprilTagDetection> matches = new ArrayList<>();
        if (detections == null) {
            return matches;
        }
        for (AprilTagDetection detection : detections) {
            if (isTagInAlliance(detection, alliance)) {
                matches.add(detection);
            }
        }
        return matches;
    }

    public void init() {
        if (mds == null || mds.hardware == null || mds.hardware.cam1 == null) {
            return;
        }

        if (mds.mode != null && mds.mode.equalsIgnoreCase("auton")) {
            camMode = MODE_BALL;
        } else {
            camMode = MODE_TAG;
        }

        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        if (visionPortal2 != null) {
            visionPortal2.close();
            visionPortal2 = null;
        }

        if (Objects.equals(camMode, MODE_TAG)) {
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
        } else if (Objects.equals(camMode, MODE_BALL)) {
            ballDetectG = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
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
                    .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
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
            builder2.setCamera(mds.hardware.cam1);
            visionPortal2 = builder2.build();
        }

        if (dashCamStream) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            if (Objects.equals(camMode, MODE_TAG) && visionPortal != null) {
                dashboard.startCameraStream(visionPortal, 0);
            } else if (Objects.equals(camMode, MODE_BALL) && visionPortal2 != null) {
                dashboard.startCameraStream(visionPortal2, 0);
            }
        }
    }

    public List<AprilTagDetection> currentDetections = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> currentBlobs = new ArrayList<>();

    public void update_april() {
        if (aprilTag != null) {
            currentDetections = aprilTag.getDetections();
        } else {
            currentDetections.clear();
        }
    }

    public void update_blobs() {
        currentBlobs.clear();
        if (ballDetectG != null) {
            currentBlobs.addAll(ballDetectG.getBlobs());
        }
        if (ballDetectP != null) {
            currentBlobs.addAll(ballDetectP.getBlobs());
        }
    }
}
