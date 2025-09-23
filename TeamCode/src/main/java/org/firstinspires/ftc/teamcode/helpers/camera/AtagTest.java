package org.firstinspires.ftc.teamcode.helpers.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;


@TeleOp(name = "Find Tags", group = "TeleOp")
public class AtagTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        aprilTags aprilTags = new aprilTags(hardwareMap);

        telemetry.addData("Status", "Ready to go!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            aprilTags.openLiveView();
            List<AprilTagDetection> currentDetections = aprilTags.getDetections();

            if (!currentDetections.isEmpty()) {
                telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());

                for (AprilTagDetection detection : currentDetections) {
                    telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
                    telemetry.addLine(String.format("  - X: %.2f", detection.ftcPose.x));
                    telemetry.addLine(String.format("  - Y: %.2f", detection.ftcPose.y));
                    telemetry.addLine(String.format("  - Z: %.2f", detection.ftcPose.z));
                }
            } else {
                telemetry.addData("Status", "No AprilTags found.");
            }

            telemetry.update();
        }

        aprilTags.close();
    }
}