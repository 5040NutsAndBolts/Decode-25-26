package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.ParentAuton;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Autonomous(name="Test", group="Autonomous")
public class TestAuto extends ParentAuton {
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
            drivetrain.robotOrientedDrive(0,0,0);
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
        requestOpModeStop();
    }
}}