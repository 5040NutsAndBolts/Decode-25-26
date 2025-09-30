package org.firstinspires.ftc.teamcode.helpers.camera;

import android.util.Size;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class aprilTags {
    private final VisionPortal vPortal;
    private final AprilTagProcessor aprilTagProcessor;
    private List<AprilTagDetection> detections;
    //as per documentation, Christin En-Hsien Tallman, comment this code.
    public aprilTags(@NonNull HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        vPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "AprilTags"))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(800,600))
                .enableLiveView(true)
                .build();
    }

    public ArrayList<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    public void close() {
        if (vPortal != null) {
            vPortal.close();
        }
    }

    public void check(){ //only important if using multiple cameras
            vPortal.getActiveCamera();
    }

    public String currState(){
            return "" + vPortal.getCameraState();
    }

}
