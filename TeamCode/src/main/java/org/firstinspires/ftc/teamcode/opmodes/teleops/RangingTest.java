package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.camera.AprilTags;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name= "RangingTest", group="Teleop")
public class RangingTest extends OpMode {

    AprilTags at;

    @Override
    public void init() {
        at = new AprilTags(hardwareMap);
    }

    boolean cameraGood;
    boolean isFar = true;

    @Override
    public void loop() {

        List<AprilTagDetection> currentDetections = AprilTags.getDetections();
        try {
            if (!currentDetections.isEmpty()) {
                telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());

                for (AprilTagDetection detection : currentDetections) {
                    telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
                    telemetry.addLine(String.format("  - Z: %.2f", detection.ftcPose.z));
                    isFar = detection.ftcPose.z <= -12;
                    if (detection.ftcPose.yaw >= 27 && detection.ftcPose.yaw <= 34.7 && detection.id == 20) {
                        cameraGood = true;
                    } else {
                        if (detection.ftcPose.yaw >= -31 && detection.ftcPose.yaw <= -26 && detection.id == 24) {
                            cameraGood = true;
                        }
                    }
                }
            } else {
                telemetry.addLine("no tags seen");
                cameraGood = false;
            }
        } catch (Exception e){
            telemetry.addLine("Camera Issue, please consult the thell bell");
        }
        telemetry.addLine("CG:  "+cameraGood);
        telemetry.addLine("iF:  " + isFar);
    }
}
