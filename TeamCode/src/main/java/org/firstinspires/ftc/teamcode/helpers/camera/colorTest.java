package org.firstinspires.ftc.teamcode.helpers.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;


@TeleOp(name = "Find Tags", group = "TeleOp")
public class colorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ColorLocate cLocate = new ColorLocate(hardwareMap);

        telemetry.addData("Status", "Ready to go!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {


            telemetry.update();
        }

        cLocate.close();
    }
}