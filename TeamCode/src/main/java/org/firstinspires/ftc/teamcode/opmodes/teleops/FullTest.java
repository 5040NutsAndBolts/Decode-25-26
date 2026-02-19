package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.mechanisms.CSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Lights;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="FullTest", group="Teleop")
public class FullTest extends OpMode {
	Launcher launcher;
	Drivetrain drivetrain;
	Lights lightBL, lightFL, lightFR, lightBR;
	PID flywheelController;
	CSensor colorSensor;
	boolean isFar = true, lastOverrideToggleInput = false, overrideToggle = false;

	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		launcher = new Launcher(hardwareMap);
		lightBL = new Lights(hardwareMap, "Mlight");
		lightFL = new Lights(hardwareMap, "Tlight");
		lightFR = new Lights(hardwareMap, "Flight");
		lightBR = new Lights(hardwareMap, "Elight");
		colorSensor = new CSensor("CSensor", hardwareMap);
		flywheelController = new PID(.006, 1e-8, 0, launcher::flywheelRPMS, 0);
	}

	@SuppressLint("DefaultLocale")
	@Override
	public void loop() {
		if(gamepad2.b && gamepad2.b != lastOverrideToggleInput)
			overrideToggle = !overrideToggle;
		lastOverrideToggleInput = gamepad2.b;
;
		try {
			if (!org.firstinspires.ftc.teamcode.helpers.camera.aprilTags.getDetections().isEmpty()) {
				for (AprilTagDetection detection : org.firstinspires.ftc.teamcode.helpers.camera.aprilTags.getDetections()) {
					if ((detection.ftcPose.yaw >= 27 && detection.ftcPose.yaw <= 34.7 && detection.id == 20) || (detection.ftcPose.yaw >= -31 && detection.ftcPose.yaw <= -26 && detection.id == 24)) {
						lightBL.setPattern(Lights.Color.BLUE);
						lightBR.setPattern(Lights.Color.BLUE);
					}else {
						lightBL.setPattern(Lights.Color.ORANGE);
						lightBR.setPattern(Lights.Color.ORANGE);
					}
					isFar = detection.ftcPose.z <= -12;
				}
			}
		} catch (Exception e){
			overrideToggle = true;
			telemetry.addLine("FORCED AUTORANGE OVERRIDE || CAMERA ERROR");
		}

		boolean idle = true;
		if(overrideToggle){
			if(gamepad2.right_trigger > 0.25){
				flywheelController.setTarget(5300);
				idle=false;
				isFar = true;
			} else if(gamepad2.left_trigger > 0.25) {
				flywheelController.setTarget(4400);
				idle=false;
				isFar = false;
			}
		}else if(gamepad2.right_trigger > 0.25){
			flywheelController.setTarget(isFar ? 5300 : 4400);
			idle=false;
		}

		if(idle) {
			launcher.setOuttakePower(.3);
			lightFL.setPattern(colorSensor.getColor());
			lightFR.setPattern(colorSensor.getColor());
		}else {
			launcher.setOuttakePower(flywheelController.autoControl());
			if(launcher.flywheelRPMS() > flywheelController.getTarget() * .95){
				gamepad2.rumble(100);
				gamepad1.rumble(100);
				telemetry.addLine("rumbling " + (isFar ? "far" : "close"));
				lightFL.setPattern(Lights.Color.WHITE);
				lightFR.setPattern(Lights.Color.WHITE);
			}else {
				lightFL.setPattern(Lights.Color.ORANGE);
				lightFR.setPattern(Lights.Color.ORANGE);
			}
		}

		drivetrain.toggleSlowMode(gamepad1.dpad_down);
		drivetrain.robotOrientedDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
		launcher.intake(gamepad1.right_trigger-gamepad1.left_trigger);
		launcher.transfer(-gamepad2.left_stick_y);
		launcher.fling(gamepad2.right_stick_y);
		drivetrain.toggleSlowMode(gamepad1.b);

		telemetry.addLine("Autoranging Override?: " + overrideToggle);
		telemetry.addLine("Launcher: \n" + launcher.toString());
		telemetry.addLine("Drivetrain: \n" + drivetrain.toString());
		telemetry.update();
	}
}
