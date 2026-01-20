package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@Autonomous(name="CameraTest", group="Autonomous")
public class CameraTest extends OpMode {
    Drivetrain drivetrain;
    aprilTags aprilTag;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        aprilTag = new aprilTags(hardwareMap);
        telemetry.addLine((drivetrain.toString() + "Initializing"));

        telemetry.update();
    }
    @Override
    public  void init_loop(){
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
    }

    @Override
    public void loop() {
        ElapsedTime timer;
        timer = new ElapsedTime();
        while(timer.seconds() < 5){
        List<AprilTagDetection> currentDetections = aprilTags.getDetections();

        if (!currentDetections.isEmpty()) {
            telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
                telemetry.addLine(String.format("  - X: %.2f", detection.ftcPose.x));
                telemetry.addLine(String.format("  - Y: %.2f", detection.ftcPose.y));
                telemetry.addLine(String.format("  - Z: %.2f", detection.ftcPose.z));
                telemetry.addLine(String.format("  - Yaw: %.2f", detection.ftcPose.yaw));
                telemetry.addLine(String.format("  - Roll: %.2f", detection.ftcPose.roll));
                telemetry.addLine(String.format("  - Pitch: %.2f", detection.ftcPose.pitch));

            }
        } else {
            telemetry.addData("Status", "No AprilTags found.");
        }
        telemetry.update();
    }
        requestOpModeStop();
}}