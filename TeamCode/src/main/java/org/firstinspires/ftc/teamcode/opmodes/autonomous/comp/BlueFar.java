package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.ParentAuton;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


@Autonomous(name="BlueFar", group="Autonomous")
public class  BlueFar extends ParentAuton {
	Drivetrain drivetrain;
	Launcher launcher;
	aprilTags aprilTag;
	double[] setTarget;
	TelemetryPacket packet;
	FtcDashboard dash;
	HashMap<String, Object> map = new HashMap<>();
	boolean kill = false;

	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		launcher = new Launcher(hardwareMap);
		aprilTag = new aprilTags(hardwareMap);
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
		map.put("", loopName);
		map.put("Kill", kill);
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

		dash.startCameraStream(aprilTag.getCameraStreamProcessor(), 0);
		sendTelemetry("Initialization", true);
	}

	@Override
	public void loop() {
		if(kill){
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
		drivetrain.robotOrientedDrive(0, 0, 0);
		launcher.fling(false);

		timer = new ElapsedTime();
		while(launcher.flywheelRPMS() < 5300 || timer.seconds() > 2){
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

		drivetrain.robotOrientedDrive(0, 0, 0);

		timer = new ElapsedTime();
		while(timer.seconds()<2.5){
			launcher.fling(false);
			launcher.intake(1);
			launcher.transfer(-1);
			sendTelemetry("Move second preload under fling", false);
		}
		launcher.intake(0);

		timer = new ElapsedTime();
		while(launcher.flywheelRPMS() < 5150 || timer.seconds() > 2){
			launcher.outtake(1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			sendTelemetry("Get up to speed", false);
		}

		timer = new ElapsedTime();
		while(timer.seconds()<1.25) {
			launcher.fling(true);
			launcher.transfer(-1);
			sendTelemetry("Launching second preload", false);
		}
		launcher.transfer(0);
		launcher.fling(false);

		setTarget[0]=15;
		while(setTarget[0] > drivetrain.getPosition()[0]){
			launcher.outtake(0);
			drivetrain.robotOrientedDrive(.2, 0, 0);
			sendTelemetry("Align with pickups", false);
		}

		setTarget[2] = -111;
		while(setTarget[2] < drivetrain.getPosition()[2]){
			drivetrain.robotOrientedDrive(0, 0, -0.2);
			sendTelemetry("Face Pochita's intake towards the\nfirst row of pickups", false);
		}

		drivetrain.robotOrientedDrive(0, 0, 0);

		setTarget[1] = 27.5;
		while(setTarget[1] > drivetrain.getPosition()[1]) {
			drivetrain.robotOrientedDrive(0, 0.25, 0);
			drivetrain.updateOdo();
			sendTelemetry("Strafe to align with artifacts", false);
		}
		requestOpModeStop();
		kill = true;

		/*

		setTarget[0] = 28;
		while(setTarget[0] > drivetrain.getPosition()[0]){
			drivetrain.robotOrientedDrive(-0.25, 0, 0);
			launcher.intake(1);
			launcher.transfer(-1);
			sendTelemetry("grrr", false);
			launcher.outtake(0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}
		//need to adjust pick up - jack
		timer = new ElapsedTime();
		while(timer.seconds()<0.2)
			drivetrain.robotOrientedDrive(0,0,0);

		timer = new ElapsedTime();
		while(timer.seconds()<0.05)
			launcher.intake(0);
		//retrieve ball

		setTarget[0] = 19;
		while(setTarget[0] < drivetrain.getPosition()[0]){
			launcher.fling(false);
			drivetrain.robotOrientedDrive(0.25, 0, 0);
			drivetrain.updateOdo();
			sendTelemetry("grrr", false);
			telemetry.update();
		}

		timer = new ElapsedTime();
		while(timer.seconds() < 0.5)
			drivetrain.robotOrientedDrive(0,0,0);

		setTarget[1] = 31;
		while(setTarget[1] < drivetrain.getPosition()[1]) {
			drivetrain.robotOrientedDrive(0, -0.4, 0);
			drivetrain.updateOdo();
			sendTelemetry("Strafe1", false);
			telemetry.update();

		}

		setTarget[2] = -5;
		while(setTarget[2] > drivetrain.getPosition()[2]){
			drivetrain.robotOrientedDrive(0, 0, 0.2);
			drivetrain.updateOdo();
			sendTelemetry("ROTATOR", false);
			telemetry.update();
		}

		setTarget[0]= -8;
		while(setTarget[0] < drivetrain.getPosition()[0]){
			launcher.outtake(0.8);
			drivetrain.robotOrientedDrive(-.2, 0, 0);
			drivetrain.updateOdo();
			sendTelemetry("move back, get up to speed", false);
		}
		drivetrain.robotOrientedDrive(0, 0, 0);

		timer = new ElapsedTime();
		while(timer.seconds() < 2) {
			launcher.intake(1);
			launcher.transfer(-1);
			launcher.fling(true);
			sendTelemetry("Launch second collected ball", false);
		}

		while(true){
			sendTelemetry("waity waity waity", false);
			drivetrain.robotOrientedDrive(0,0,0);
			drivetrain.neutral();
			drivetrain.updateOdo();
		}
		/*requestOpModeStop();*/
	}
}