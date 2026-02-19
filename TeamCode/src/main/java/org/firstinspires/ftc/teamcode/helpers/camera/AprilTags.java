package org.firstinspires.ftc.teamcode.helpers.camera;

import android.util.Size;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AprilTags {
    private final VisionPortal vPortal;
    private static AprilTagProcessor aprilTagProcessor;
    private final CameraStreamProcessor cameraStreamProcessor;

    public AprilTags(@NonNull HardwareMap hardwareMap) {

        cameraStreamProcessor = new CameraStreamProcessor();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        vPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "AprilTags"))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(640,480))
                .addProcessor(cameraStreamProcessor)
                .setAutoStartStreamOnBuild(true)
                .enableLiveView(true)
                .build();
    }

    public static ArrayList<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    public CameraStreamProcessor getCameraStreamProcessor() {
        return cameraStreamProcessor;
    }

    public void close() {
        if (vPortal != null) {
            vPortal.close();
        }
    }

    public Map<String, Object> getCameraTelemetry(boolean inInit) { //scans the currently detected tags and puts all the telemetry to dashboard

        HashMap<String, Object> map = new HashMap<>(); //object used to store and name sets of data

        List<AprilTagDetection> currentDetections = AprilTags.getDetections();

        if (!currentDetections.isEmpty()) {

            for (AprilTagDetection detection : currentDetections) { //the packet.putAll only takes map objects
                map.put("Detecting: ", detection.id);
                map.put("Z: ", detection.ftcPose.z);
                map.put("Pitch: ", detection.ftcPose.pitch);
                map.put("Yaw: ", detection.ftcPose.yaw);

            }
        }
        return map;
    }

    public void check(){ //only important if using multiple cameras
            vPortal.getActiveCamera();
    }
    public void pause(){
        vPortal.stopStreaming();
    }
    public void resume(){
        vPortal.resumeStreaming();
    }
    public String currState(){
            return "" + vPortal.getCameraState();
    }

}
