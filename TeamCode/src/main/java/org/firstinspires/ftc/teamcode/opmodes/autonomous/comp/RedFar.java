package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Lights;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.ParentAuton;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


@Autonomous(name="RedFar", group="Autonomous")
public class RedFar extends ParentAuton {
	Drivetrain drivetrain;
	Launcher launcher;
	aprilTags aprilTag;
	double[] setTarget;
	TelemetryPacket packet;
	FtcDashboard dash;
	HashMap<String, Object> map = new HashMap<>();

	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		launcher = new Launcher(hardwareMap);
		aprilTag = new aprilTags(hardwareMap);
		telemetry.addLine((drivetrain + "I"));
		drivetrain.updateOdo();
		telemetry.update();
		setTarget = new double[]{
				5.5, 0, 0
		};
		dash = FtcDashboard.getInstance();
		packet=new TelemetryPacket();
	}

	private void sendTelemetry (String loopName) {
		telemetry.addLine(loopName);
		telemetry.addLine("Flywheel speed: " + launcher.flywheelRPMS());
		telemetry.addLine("Odo: " + drivetrain.getPosition()[0] + " , " + drivetrain.getPosition()[1] + " , " + drivetrain.getPosition()[2]);
		telemetry.update();
		packet.clearLines();
		Map<String, Object> map = launcher.getPIDTelemetry(false);
		map.put("Odo X", drivetrain.getPosition()[0]);
		map.put("Odo Y", drivetrain.getPosition()[1]);
		map.put("Odo R", drivetrain.getPosition()[2]);
		map.put("", loopName);
		packet.putAll(map);
		dash.sendTelemetryPacket(packet);
	}


	@Override
	public void init_loop() {
		super.init_loop();
		packet.clearLines();
		packet.putAll(launcher.getPIDTelemetry(true));
		dash.sendTelemetryPacket(packet);

		List<AprilTagDetection> currentDetections = aprilTags.getDetections();

		if (!currentDetections.isEmpty()) {
			telemetry.addData("Status", "Found %d AprilTags!", currentDetections.size());

			for (AprilTagDetection detection : currentDetections) {
				telemetry.addLine(String.format("Found Tag ID: %d", detection.id));
				telemetry.addLine(String.format("  - X: %.2f", detection.ftcPose.x));
				telemetry.addLine(String.format("  - Y: %.2f", detection.ftcPose.y));
				telemetry.addLine(String.format("  - Z: %.2f", detection.ftcPose.z));
				map.put("Pitch", detection.ftcPose.pitch);
				map.put("Yaw", detection.ftcPose.yaw);
				packet.putAll(map);
			}
		} else {
			telemetry.addData("Status", "No AprilTags found.");
		}


		telemetry.addLine((drivetrain.toString() + "IL"));
		telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
		telemetry.addLine(""+drivetrain.getPosition()[0]);
		telemetry.addLine(""+drivetrain.getPosition()[1]);
		telemetry.update();
	}

	@Override
	public void loop() {
		ElapsedTime timer;
		drivetrain.resetOdo();
		launcher.transfer(-1);
		launcher.outtake(0.76);

		while(setTarget[0] > drivetrain.getPosition()[0]){
			drivetrain.robotOrientedDrive(0.2, 0, 0);
			drivetrain.updateOdo();
			sendTelemetry("first move loop");
		}

		timer = new ElapsedTime();
		while(timer.seconds()<0.2){
			sendTelemetry("Rotating");
			drivetrain.robotOrientedDrive(0,0,-0.12);
		}

		drivetrain.robotOrientedDrive(0, 0, 0);
		launcher.fling(false);

		while(timer.seconds()<2){
			launcher.transfer(-1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			sendTelemetry("Transfering");
		}

		timer = new ElapsedTime();
		while(timer.seconds()<0.06){
			sendTelemetry("Rotating");
			drivetrain.robotOrientedDrive(0,0,0.1);
		}

		launcher.fling(true);

		timer = new ElapsedTime();
		while(timer.seconds()<2.5){
			launcher.transfer(-1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			sendTelemetry("Send ball to fling");
		}

		launcher.fling(false);
		drivetrain.robotOrientedDrive(0, 0, 0);

		timer = new ElapsedTime();
		while (timer.seconds() < 2.5) {
			launcher.intake(1);
			sendTelemetry("Intaking");
		}

		launcher.fling(true);

		timer = new ElapsedTime();
		while (timer.seconds() < .7) {
			launcher.fling(true);
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
		}
		setTarget[0] = 17;
		while (setTarget[0] > drivetrain.getPosition()[0]) {
			launcher.outtake(0);
			drivetrain.robotOrientedDrive(.2, 0, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString()));
			telemetry.update();
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
		}

		drivetrain.robotOrientedDrive(0, 0, 0);

		setTarget[2] = 105;
		while (setTarget[2] > drivetrain.getPosition()[2]) {
			drivetrain.robotOrientedDrive(0, 0, 0.25);
			drivetrain.updateOdo();
			launcher.intake(1);
			sendTelemetry("rotator");
		}
		drivetrain.robotOrientedDrive(0, 0, 0);

		setTarget[1] = 3;

		while (setTarget[1] < drivetrain.getPosition()[1]) {
			drivetrain.robotOrientedDrive(0, -0.3, 0);
			drivetrain.updateOdo();
			launcher.intake(1);
			launcher.fling(false);
			launcher.transfer(-1);
			sendTelemetry("Align with artifacts");
		}
		//restart pinpoint
		Pose2D pose2D = new Pose2D(DistanceUnit.INCH, drivetrain.getPosition()[0], drivetrain.getPosition()[1], AngleUnit.DEGREES,drivetrain.getPosition()[2]);
		drivetrain.resetOdo();
		drivetrain.odo.set(pose2D);
		setTarget[0] = -17.5;
		while (setTarget[0] < drivetrain.getPosition()[0]) {
			drivetrain.robotOrientedDrive(-0.25, 0, 0);
			launcher.fling(false);
			launcher.intake(1);
			launcher.transfer(-1);
			launcher.outtake(0);
			sendTelemetry("Intake artifacts");
		}
		pose2D = new Pose2D(DistanceUnit.INCH, drivetrain.getPosition()[0], drivetrain.getPosition()[1], AngleUnit.DEGREES,drivetrain.getPosition()[2]);
		drivetrain.resetOdo();
		drivetrain.odo.set(pose2D);

		setTarget[0] = 10;
		while (setTarget[0] > drivetrain.getPosition()[0]) {
			launcher.fling(false);
			drivetrain.robotOrientedDrive(0.25, 0, 0);
			sendTelemetry("Move back after intaking artifacts");
		}

		drivetrain.robotOrientedDrive(0, 0, 0);;

		while (drivetrain.getPosition()[1] > -20) {
			drivetrain.robotOrientedDrive(0, 0.3, 0);
			drivetrain.updateOdo();
			sendTelemetry("Strafe back to wall ");
			launcher.intake(0);
			launcher.transfer(0);
		}

		while (drivetrain.getPosition()[0] < 20) {
			drivetrain.robotOrientedDrive(.3, 0, 0);
			drivetrain.updateOdo();
			sendTelemetry("Return to shooting position");
		}

		while (drivetrain.getPosition()[2] > -105) {
			drivetrain.robotOrientedDrive(0, 0, -.2);
			launcher.outtake(.9);
			drivetrain.updateOdo();
			sendTelemetry("Rotate");
		}

		drivetrain.robotOrientedDrive(0, 0, 0);

		timer = new ElapsedTime();
		while(timer.seconds() < .8) {
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.outtake(.9);
			launcher.fling(true);
			sendTelemetry("Launch collected ball 1");
		}

		timer = new ElapsedTime();
		while(timer.seconds() < .8) {
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.intake(1);
			launcher.transfer(-1);
			launcher.outtake(.9);
			launcher.fling(false);
			sendTelemetry("Get up to speed and bring ball to front");
		}

		timer = new ElapsedTime();
		while(timer.seconds() < .8) {
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.outtake(.9);
			launcher.fling(true);
			sendTelemetry("Launch collected ball 2");
		}

		while(true){
			sendTelemetry("waity waity waity");
			drivetrain.robotOrientedDrive(0,0,0);
			drivetrain.neutral();
			drivetrain.updateOdo();
		}



			/*

			setTarget[2] = 0;
			while (setTarget[2] < drivetrain.getPosition()[2]) {
				drivetrain.robotOrientedDrive(0, 0, 0.2);
				drivetrain.updateOdo();
				telemetry.update();
				packet.clearLines();
				packet.putAll(launcher.getPIDTelemetry(false));
				dash.sendTelemetryPacket(packet);
			}

			setTarget[0] = 1.5;
			while (setTarget[0] < drivetrain.getPosition()[0]) {
				launcher.outtake(0.8);
				drivetrain.robotOrientedDrive(-.2, 0, 0);
				drivetrain.updateOdo();
				telemetry.update();
				packet.clearLines();
				packet.putAll(launcher.getPIDTelemetry(false));
				dash.sendTelemetryPacket(packet);
			}


			timer = new ElapsedTime();
			while (timer.seconds() < 0.5) {
				launcher.transfer(-1);
				packet.clearLines();
				packet.putAll(launcher.getPIDTelemetry(false));
				dash.sendTelemetryPacket(packet);
			}

			launcher.fling(true);

			timer = new ElapsedTime();
			while (timer.seconds() < 2) {
				launcher.intake(1);
				launcher.transfer(-1);
				packet.clearLines();
				packet.putAll(launcher.getPIDTelemetry(false));
				dash.sendTelemetryPacket(packet);
			}
			timer = new ElapsedTime();
			while (timer.seconds() < 3) {
				launcher.fling(true);
				packet.clearLines();
				packet.putAll(launcher.getPIDTelemetry(false));
				dash.sendTelemetryPacket(packet);
			}*/


			//requestOpModeStop();
				}}