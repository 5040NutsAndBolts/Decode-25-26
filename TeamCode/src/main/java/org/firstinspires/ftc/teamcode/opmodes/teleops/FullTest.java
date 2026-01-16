package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Lights;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="FullTest", group="Teleop")
public class FullTest extends OpMode {
	Launcher la;
	Drivetrain dt;
	aprilTags at;
	Lights lH, lL;
	boolean cameraGood;

	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
		la = new Launcher(hardwareMap);
		lH = new Lights(hardwareMap, "Lights High");
		lL = new Lights(hardwareMap, "Lights Low");
		at = new aprilTags(hardwareMap);
	}

	@Override
	public void loop() {
		dt.robotOrientedDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
		dt.updateOdo();

		la.intake(gamepad1.left_trigger-gamepad1.right_trigger);

		la.transfer(-gamepad2.left_stick_y);

		la.fling(gamepad2.b);

		dt.toggleSlowMode(gamepad1.b);

		la.setOuttakePower(gamepad2.left_trigger > 0.25 ? .95 : .2);
		if(gamepad2.right_trigger > 0.25){
			la.setOuttakePower(.75);
			gamepad2.rumble(200);
			gamepad1.rumble(200);
		}

		la.fling(gamepad2.a);

		dt.toggleSlowMode(gamepad1.dpad_down);

		List<AprilTagDetection> currentDetections = aprilTags.getDetections();

		if(la.flywheelRPMS() > 5100) {
			gamepad2.rumble(200);
			gamepad1.rumble(200);
			telemetry.addLine("rumbling, far");
		} else {
			if(!cameraGood){
				lH.setPattern(la.topColor());
				lL.setPattern(la.lowColor());
			}
		}

		if (!currentDetections.isEmpty()) {
			telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());

			for (AprilTagDetection detection : currentDetections) {
				telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
				telemetry.addLine(String.format("  - Yaw: %.2f", detection.ftcPose.yaw));
				if (detection.ftcPose.yaw >= 27 && detection.ftcPose.yaw <= 34.7 && detection.id == 20) {
					lH.setPattern(Lights.Color.BLUE);
					cameraGood = true;
				}	else
						lH.setPattern(Lights.Color.WHITE);
			}
		} else {
			telemetry.addLine("no tags seen");
			cameraGood = false;
		}

		telemetry.addLine("Launcher: \n" + la.toString());
		telemetry.addLine("Drivetrain: \n" + dt.toString());
		telemetry.update();
	}
}
