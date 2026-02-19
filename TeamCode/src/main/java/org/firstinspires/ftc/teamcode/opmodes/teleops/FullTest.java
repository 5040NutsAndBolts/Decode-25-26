package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import org.firstinspires.ftc.teamcode.mechanisms.CSensor;
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
	Lights lM, lT, lF, lE;
	boolean cameraGood;
	PID pid;
	CSensor cS;

	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
		la = new Launcher(hardwareMap);
		lM = new Lights(hardwareMap, "Mlight");
		lT = new Lights(hardwareMap, "Tlight");
		lF = new Lights(hardwareMap, "Flight");
		lE = new Lights(hardwareMap, "Elight");
		at = new aprilTags(hardwareMap);
		pid = new PID(.006, 1e-8, 0, la::flywheelRPMS, 0);
		cS = new CSensor("CSensor", hardwareMap);
	}

	@SuppressLint("DefaultLocale")
	@Override
	public void loop() {
		dt.robotOrientedDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
		dt.updateOdo();

		la.intake(gamepad1.right_trigger-gamepad1.left_trigger);

		la.transfer(-gamepad2.left_stick_y);

		la.fling(gamepad2.right_stick_y);

		dt.toggleSlowMode(gamepad1.b);

		if(gamepad2.right_trigger > 0.25){
			pid.setTarget(5300);
			if(la.flywheelRPMS() > pid.getTarget() * .97){
				gamepad2.rumble(100);
				gamepad1.rumble(100);
				telemetry.addLine("rumbling, far");
				lT.setPattern(Lights.Color.WHITE);
				lF.setPattern(Lights.Color.WHITE);
			}
			la.setOuttakePower(pid.autoControl());
		} else if(gamepad2.left_trigger > 0.25) {
			pid.setTarget(4400);
			if(la.flywheelRPMS() > pid.getTarget() * .97){
				gamepad2.rumble(100);
				gamepad1.rumble(100);
				telemetry.addLine("rumbling, close");
				lT.setPattern(Lights.Color.WHITE);
				lF.setPattern(Lights.Color.WHITE);
			}
			la.setOuttakePower(pid.autoControl());
		} else {
			la.setOuttakePower(.3);
			lT.setPattern(cS.getColor());
			lF.setPattern(cS.getColor());
		}

		dt.toggleSlowMode(gamepad1.dpad_down);

		List<AprilTagDetection> currentDetections = aprilTags.getDetections();
		try {
			if (!currentDetections.isEmpty()) {
				telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());
				for (AprilTagDetection detection : currentDetections) {
					telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
					telemetry.addLine(String.format("  - Yaw: %.2f", detection.ftcPose.yaw));
					if (detection.ftcPose.yaw >= 27 && detection.ftcPose.yaw <= 34.7 && detection.id == 20) {
						lM.setPattern(Lights.Color.BLUE);
						lE.setPattern(Lights.Color.BLUE);
						cameraGood = true;
					} else {
						if (detection.ftcPose.yaw >= -31 && detection.ftcPose.yaw <= -26 && detection.id == 24) {
							lM.setPattern(Lights.Color.BLUE);
							lE.setPattern(Lights.Color.BLUE);
							cameraGood = true;
						}
					}
				}
			} else {
				lM.setPattern(Lights.Color.ORANGE);
				lE.setPattern(Lights.Color.ORANGE);
				telemetry.addLine("no tags seen");
				cameraGood = false;
			}
		} catch (Exception e){
			telemetry.addLine("Camera Issue, please consult the thelly belly");
		}

		telemetry.addLine("CS: \n" + cS.toString());
		telemetry.addLine("Launcher: \n" + la.toString());
		telemetry.addLine("Drivetrain: \n" + dt.toString());
		telemetry.update();
	}
}
