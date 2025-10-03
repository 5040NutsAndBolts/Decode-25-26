package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Path;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import java.util.ArrayList;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;


@Autonomous(name = "Pinpoint Tags", group = "Autonomous")
public class movingTag extends ParentAuton {
    aprilTags atag;
    @Override
    public void init() {
        super.init();

        Path sideways = new Path("sideways");
        sideways.queueStates(
                new ArrayList<Object[]>() {{
                    add(new Object[]{Drivetrain.class, new Object[]{2, 2, 0}, new Object[]{1,5}});
                }}
        );

        sideways.addMechanism(new Drivetrain(hardwareMap));


     atag = new aprilTags(hardwareMap);

     atag.check();

        List<AprilTagDetection> currentDetections = atag.getDetections();

            if (!currentDetections.isEmpty()) {
                telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());

                for (AprilTagDetection detection : currentDetections) {
                    telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
                    telemetry.addLine(String.format("  - X: %.2f", detection.ftcPose.x));
                    telemetry.addLine(String.format("  - Y: %.2f", detection.ftcPose.y));
                    telemetry.addLine(String.format("  - Z: %.2f", detection.ftcPose.z));

                    if (detection.ftcPose.x > 2) {
                        sideways.queueStates(
                                new ArrayList<Object[]>() {{
                                    add(new Object[]{Drivetrain.class, new Object[]{1, 1, 0}, new Object[]{1,5}});
                                }}
                        );
                        hold(1000);
                    }
                    if (detection.ftcPose.x < -2) {
                        sideways.queueStates(
                                new ArrayList<Object[]>() {{
                                    add(new Object[]{Drivetrain.class, new Object[]{-1, -1, 0}, new Object[]{1,5}});
                                }}
                        );
                        hold(1000);
                    } else {
                        telemetry.addData("Status", "No AprilTags found. Searching...");

                        sideways.queueStates(
                                new ArrayList<Object[]>() {{
                                    add(new Object[]{Drivetrain.class, new Object[]{2, 2, 0}, new Object[]{1,5}});
                                }}
                        );
                        hold(1000);
                    }
                }
            }

    }
}