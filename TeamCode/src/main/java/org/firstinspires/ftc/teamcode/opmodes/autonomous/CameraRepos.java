package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.ParentAuton;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Autonomous(name="Camera Reposition", group="Autonomous")
public class CameraRepos extends ParentAuton {
    Drivetrain drivetrain;
    Odometry odo;
    Launcher launcher;
    aprilTags aprilTags;


    double[] setTarget;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        aprilTags = new aprilTags(hardwareMap);
        launcher = new Launcher(hardwareMap);
        telemetry.addLine((drivetrain + "I"));
        drivetrain.updateOdo();
        odo = new Odometry(hardwareMap, 0, 0);
        telemetry.update();
        setTarget = new double[]{
                0, -7.75, 0
        };
    }
    @Override
    public void init_loop() {
        super.init_loop();

        telemetry.addLine((drivetrain.toString() + "IL"));
        telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
        telemetry.addLine(""+drivetrain.getPosition()[0]);
        telemetry.addLine(""+drivetrain.getPosition()[1]);
        odo.update();
        telemetry.update();
    }


    @Override
    public void loop() {
            List<AprilTagDetection> currentDetections = aprilTags.getDetections();
            AprilTagDetection best = null;
            if (!currentDetections.isEmpty()) {
                telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());

                for (AprilTagDetection detection : currentDetections) {
                    telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
                    telemetry.addLine(String.format("  - X: %.2f", detection.ftcPose.x));
                    telemetry.addLine(String.format("  - Y: %.2f", detection.ftcPose.y));
                    telemetry.addLine(String.format("  - Z: %.2f", detection.ftcPose.z));
                    telemetry.addLine(String.format("  - Z: %.2f", detection.ftcPose.yaw));
                    best = detection;
                }
            } else {
                telemetry.addData("Status", "No AprilTags found.");
            }
            while(best != null){
                while(best.ftcPose.yaw >= 30)
                    drivetrain.robotOrientedDrive(0,0.1,0);
                while(best.ftcPose.yaw <= -30)
                    drivetrain.robotOrientedDrive(0,-0.1,0);
                if(best.ftcPose.yaw > -30 && best.ftcPose.yaw < 30)
                    drivetrain.robotOrientedDrive(0,0,360);
            }
            telemetry.update();
        }
    }
