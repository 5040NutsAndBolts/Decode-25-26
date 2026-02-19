package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.camera.AprilTags;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;


@Autonomous(name="BlueFar", group="Autonomous")
public class  BlueFar extends OpMode {
	Drivetrain drivetrain;
	Launcher launcher;
	AprilTags aprilTag;
	double[] setTarget;
	TelemetryPacket packet;
	FtcDashboard dash;
	HashMap<String, Object> map = new HashMap<>();
	boolean suicide = false;

	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		launcher = new Launcher(hardwareMap);
		aprilTag = new AprilTags(hardwareMap);
		telemetry.addLine((drivetrain + "I"));
		drivetrain.updateOdo();
		telemetry.update();
		setTarget = new double[]{
				4.5, 0, 0
		};
		dash = FtcDashboard.getInstance();
		packet=new TelemetryPacket();
	}

	private void sendTelemetry (String loopName, boolean inInit) {
		drivetrain.updateOdo();
		packet.clearLines();
		HashMap<String, Object> map = new HashMap<>();
		map.put("Name", loopName);
		map.put("Suicide", suicide);
		map.put("Odo X", drivetrain.getPosition()[0]);
		map.put("Odo Y", drivetrain.getPosition()[1]);
		map.put("Odo R", drivetrain.getPosition()[2]);
		map.put("Vel X", drivetrain.odo.getPinpoint().getVelocity().getX(DistanceUnit.INCH));
		map.put("Vel Y", drivetrain.odo.getPinpoint().getVelocity().getY(DistanceUnit.INCH));
		map.put("Vel R", drivetrain.odo.getPinpoint().getVelocity().getHeading(AngleUnit.DEGREES));
		map.putAll(launcher.getPIDTelemetry(inInit));
		packet.putAll(map);
		dash.sendTelemetryPacket(packet);
	}

	@Override
	public void init_loop() {
		super.init_loop();
		packet.clearLines();
		packet.putAll(launcher.getPIDTelemetry(true));
		dash.sendTelemetryPacket(packet);

		List<AprilTagDetection> currentDetections = AprilTags.getDetections();

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

		dash.startCameraStream(aprilTag.getCameraStreamProcessor(), 0);
		sendTelemetry("Initialization", true);
	}

	@Override
	public void loop() {
		if(suicide){
			sendTelemetry("Stopping", true);
			drivetrain.robotOrientedDrive(0,0,0);
			launcher.transfer(0);
			launcher.outtake(0);
			launcher.intake(0);
			launcher.fling(false);
			return;
		}
		ElapsedTime timer;
		drivetrain.resetOdo();
		launcher.transfer(-1);
		launcher.outtake(0.8);

		while(setTarget[0] > drivetrain.getPosition()[0]){
			drivetrain.robotOrientedDrive(0.2, 0, 0);
			sendTelemetry("Move forward to shoot first preload", false);
		}
		drivetrain.robotOrientedDrive(0,0,0);
		launcher.fling(false);

		timer = new ElapsedTime();
		while(timer.seconds() <.13) {
			launcher.outtake(1);
			launcher.transfer(-1);
			drivetrain.robotOrientedDrive(0, 0, .1);
			sendTelemetry("Fix shot slightly and get up to speed", false);
		}

		timer = new ElapsedTime();
		while(launcher.flywheelRPMS() < 5300 && timer.seconds() < 2){
			launcher.outtake(1);
			launcher.transfer(-1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			sendTelemetry("Get up to speed", false);
		}

		timer = new ElapsedTime();
		while(timer.seconds()<1){
			launcher.transfer(-1);
			launcher.fling(true);
			drivetrain.robotOrientedDrive(0, 0, 0);
			sendTelemetry("Shoot first preload", false);
		}
		launcher.fling(false);
		drivetrain.robotOrientedDrive(0, 0, 0);

		timer = new ElapsedTime();
		while(timer.seconds()<2.5){
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.fling(false);
			launcher.intake(1);
			launcher.transfer(-1);
			sendTelemetry("Move second preload under fling", false);
		}
		launcher.intake(0);

		timer = new ElapsedTime();
		while(launcher.flywheelRPMS() < 5250 || timer.seconds() > 2){
			launcher.outtake(1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			sendTelemetry("Get up to speed", false);
			launcher.fling(false);
		}

		timer = new ElapsedTime();
		while(timer.seconds()<1.75) {
			launcher.fling(true);
			launcher.transfer(-1);
			sendTelemetry("Launching second preload", false);
		}
		launcher.transfer(0);
		launcher.fling(false);

		setTarget[0]=16;
		while(setTarget[0] > drivetrain.getPosition()[0]){
			launcher.outtake(0);
			drivetrain.robotOrientedDrive(.2, 0, 0);
			sendTelemetry("Align with pickups", false);
			launcher.fling(false);
		}

		setTarget[2] = -111;
		while(setTarget[2] < drivetrain.getPosition()[2]){
			drivetrain.robotOrientedDrive(0, 0, -0.2);
			launcher.fling(false);
			sendTelemetry("Face Pochita's intake towards the\nfirst row of pickups", false);
		}

		drivetrain.robotOrientedDrive(0, 0, 0);

		setTarget[1] = 28;
		while(setTarget[1] > drivetrain.getPosition()[1]) {
			drivetrain.robotOrientedDrive(0, 0.25, 0);
			drivetrain.updateOdo();
			sendTelemetry("Strafe to align with artifacts", false);
		}
		drivetrain.robotOrientedDrive(0,0,0);

		setTarget[0] = 31.5;
		while(setTarget[0] > drivetrain.getPosition()[0]){
			drivetrain.robotOrientedDrive(-0.25, 0, 0);
			launcher.intake(1);
			launcher.transfer(-1);
			sendTelemetry("Pick up field artifacts", false);
		}
		launcher.intake(0);

		setTarget[0] = 20;
		while(setTarget[0] < drivetrain.getPosition()[0]){
			drivetrain.robotOrientedDrive(0.25, 0, 0);
			launcher.outtake(1);
			sendTelemetry("Backing up from spike mark", false);
		}

		setTarget[2] = -1.5;
		while(setTarget[2] > drivetrain.getPosition()[2]){
			drivetrain.robotOrientedDrive(0, 0, 0.22);
			launcher.outtake(1);
			sendTelemetry("Aim to shoot pick ups", false);
		}

		setTarget[1] = 31;
		while(setTarget[1] < drivetrain.getPosition()[1]) {
			drivetrain.robotOrientedDrive(0, -0.4, 0);
			launcher.outtake(1);
			sendTelemetry("Strafe back into far shooting position", false);
		}

		setTarget[0]= -2;
		timer = new ElapsedTime();
		while(setTarget[0] < drivetrain.getPosition()[0] && timer.seconds() <1){
			drivetrain.robotOrientedDrive(-.2, 0, 0);
			launcher.outtake(1);
			sendTelemetry("Move back", false);
		}

		drivetrain.robotOrientedDrive(0, 0, 0);
		timer = new ElapsedTime();
		while(launcher.flywheelRPMS() < 5300 && timer.seconds() < 2){
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.outtake(1);
			sendTelemetry("Getting up to speed", false);
		}

		timer = new ElapsedTime();
		while(timer.seconds() < 1.75) {
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.fling(true);
			launcher.outtake(1);
			sendTelemetry("Shooting first pick up", false);
		}

		timer = new ElapsedTime();
		while(timer.seconds()<2.5){
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.fling(false);
			launcher.intake(1);
			launcher.transfer(-1);
			launcher.outtake(1);
			sendTelemetry("Move second preload under fling", false);
		}
		launcher.intake(0);

		timer = new ElapsedTime();
		while(timer.seconds() < 1.75) {
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.fling(true);
			sendTelemetry("Shooting second pick up", false);
		}

		timer.reset();
		while(timer.seconds() < 2) {
			drivetrain.robotOrientedDrive(.3,0,0);
			launcher.fling(false);
			launcher.outtake(0);
			launcher.intake(0);
			launcher.transfer(0);
			sendTelemetry("Moving off line", false);
		}

		drivetrain.robotOrientedDrive(0,0,0);
		requestOpModeStop();
		suicide = true;
	}
}