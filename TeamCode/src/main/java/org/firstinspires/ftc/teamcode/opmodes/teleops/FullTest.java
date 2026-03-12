package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.camera.AprilTags;
import org.firstinspires.ftc.teamcode.mechanisms.CSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Lights;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="FullTest", group="Teleop")
public class FullTest extends OpMode {

	FtcDashboard dash;
	TelemetryPacket packet;
	Launcher launcher;
	Drivetrain drivetrain;
	Lights lightBL, lightFL, lightFR, lightBR;
	PID flywheelController;
	CSensor colorSensor;
	AprilTags at;
	byte isFar = 0;
	boolean lastOverrideToggleInput = false, overrideToggle = false;
	final int FAR_RPMS = 4100, CLOSE_LINE_RPMS = 3450, CLOSE_POINT_RPMS = 3250;
	int currentBestRPMS = 0;
	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		launcher = new Launcher(hardwareMap);
		lightBL = new Lights(hardwareMap, "Mlight");
		lightFL = new Lights(hardwareMap, "Tlight");
		lightFR = new Lights(hardwareMap, "Flight");
		lightBR = new Lights(hardwareMap, "Elight");
		colorSensor = new CSensor("CSensor", hardwareMap);
		at = new AprilTags(hardwareMap);
		flywheelController = new PID(.0075, 3e-8, 0, launcher::flywheelRPMS, 0);
		packet = new TelemetryPacket();
		dash = FtcDashboard.getInstance();
	}

	private double dist (double x1, double y1, double x2, double y2) {
		return Math.sqrt(
				(x2 - x1) * (x2 - x1) +
				(y2-y1) * (y2-y1)
		);
	}

	@SuppressLint("DefaultLocale")
	@Override
	public void loop() {
		if(gamepad2.x && gamepad2.x != lastOverrideToggleInput)
			overrideToggle = !overrideToggle;
		lastOverrideToggleInput = gamepad2.x;
;
		List<AprilTagDetection> currentDetections = AprilTags.getDetections();
		try {
			if (!currentDetections.isEmpty()) {
				telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());
				for (AprilTagDetection detection : currentDetections) {
					telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
					telemetry.addLine(String.format("  - Yaw: %.2f", detection.ftcPose.yaw));
					telemetry.addLine(String.format("  - Z: %.2f", detection.ftcPose.z));
					packet.addLine(String.format("  - Yaw: %.2f", detection.ftcPose.yaw));
					packet.addLine(String.format("  - Z: %.2f", detection.ftcPose.z));
					//yawL, yawH, Z
					double[] RL = {-5.5, -7.5, -7.3};
					double[] RP = {4,5.5,-2.8};
					double[] BP = {1.8, 3, -2.7};
					double[] BL = {-2.5,-4,-7.2};

					//0: far 1: point 2: line
					//blue id is 20
					if(detection.ftcPose.z <= -9)
						isFar = 0;
					else {
						if(detection.id == 20){
							double pDist = dist((BP[0] + BP[1] / 2), BP[2], detection.ftcPose.yaw, detection.ftcPose.z);
							double lDist = dist((BL[0] + BL[1] / 2), BL[2], detection.ftcPose.yaw, detection.ftcPose.z);
							isFar = (byte) (pDist < lDist ? 1 : 2);
						} else if(detection.id == 24){
							double pDist = dist((RP[0] + RP[1] / 2), RP[2], detection.ftcPose.yaw, detection.ftcPose.z);
							double lDist = dist((RL[0] + RL[1] / 2), RL[2], detection.ftcPose.yaw, detection.ftcPose.z);
							isFar = (byte) (pDist < lDist ? 1 : 2);
						}
						else isFar = 2;
					}

					switch (isFar) {
						case 0:
							currentBestRPMS = FAR_RPMS;
						case 1:
							currentBestRPMS = CLOSE_POINT_RPMS;
						case 2:
							currentBestRPMS = CLOSE_LINE_RPMS;
						default:
							currentBestRPMS = FAR_RPMS;
					}

					if ((((detection.ftcPose.yaw >= 27 && detection.ftcPose.yaw <= 34.7 && detection.id == 20) || (detection.ftcPose.yaw >= -31 && detection.ftcPose.yaw <= -26 && detection.id == 24))) || ((detection.ftcPose.yaw >= -6 && detection.ftcPose.yaw <= -4 && detection.id == 20) || (detection.ftcPose.yaw >= -15.8 && detection.ftcPose.yaw <= -13.8 && detection.id == 24))) {
						lightBL.setPattern(Lights.Color.BLUE);
						lightBR.setPattern(Lights.Color.BLUE);
					}else {
						lightBL.setPattern(Lights.Color.ORANGE);
						lightBR.setPattern(Lights.Color.ORANGE);
					}
				}
			}
		} catch (Exception e){
			overrideToggle = true;
			telemetry.addLine("FORCED AUTORANGE OVERRIDE || CAMERA ERROR");
			telemetry.addLine(e.getMessage());
			lightBL.setPattern(Lights.Color.OFF);
			lightBR.setPattern(Lights.Color.OFF);
		}

		boolean idle = true;
		if(overrideToggle){
			if(gamepad2.right_trigger > 0.25){
				flywheelController.setTarget(FAR_RPMS);
				idle=false;
			} else if(gamepad2.left_trigger > 0.25) {
				flywheelController.setTarget(CLOSE_LINE_RPMS);
				idle=false;
			}
		}else if(gamepad2.right_trigger > 0.25){
			flywheelController.setTarget(currentBestRPMS);
			idle=false;
		}

		if(idle) {
			launcher.setOuttakePower(.3);
			lightFL.setPattern(colorSensor.getColor());
			lightFR.setPattern(colorSensor.getColor());
		}else {
			double power = flywheelController.autoControl();
			launcher.setOuttakePower(power);
			if(launcher.flywheelRPMS() > flywheelController.getTarget() * .96){
				gamepad2.rumble(150);
				gamepad1.rumble(150);
				telemetry.addLine("rumbling " + isFar);
				lightFL.setPattern(Lights.Color.WHITE);
				lightFR.setPattern(Lights.Color.WHITE);
			}else {
				telemetry.addLine("BAD RPM RANGE");
				lightFL.setPattern(Lights.Color.OFF);
				lightFR.setPattern(Lights.Color.OFF);
			}
		}

		drivetrain.toggleSlowMode(gamepad1.dpad_down);
		drivetrain.robotOrientedDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
		launcher.intake(gamepad1.right_trigger-gamepad1.left_trigger);
		launcher.transfer(-gamepad2.left_stick_y);
		launcher.fling(gamepad2.right_stick_y);
		drivetrain.toggleSlowMode(gamepad1.b);



		//telemetry.addLine("Autoranging Override?: " + overrideToggle);
		telemetry.addLine("isFar: " + isFar);
		telemetry.addLine("CBRPMS: " + currentBestRPMS);
		//telemetry.addLine("Launcher: \n" + launcher.toString());
		//telemetry.addLine("Drivetrain: \n" + drivetrain.toString());
		telemetry.update();
		dash.sendTelemetryPacket(packet);
		packet.clearLines();
	}
}
