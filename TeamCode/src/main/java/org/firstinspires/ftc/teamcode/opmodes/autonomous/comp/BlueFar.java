package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.camera.aprilTags;
import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.ParentAuton;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


@Autonomous(name="BlueFar", group="Autonomous")
public class BlueFar extends ParentAuton {
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
				4.5, 0, 0
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

		dash.startCameraStream(aprilTag.getCameraStreamProcessor(), 0);
		sendTelemetry("Chud");
		telemetry.addLine((drivetrain.toString() + "IL"));
		telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
		telemetry.addLine(""+drivetrain.getPosition()[0]);
		telemetry.addLine(""+drivetrain.getPosition()[1]);
		telemetry.update();
	}

	@Override
	public void loop() {
		sendTelemetry("grrr");
		boolean shouldClose = false;
		ElapsedTime timer;
		drivetrain.resetOdo();
		launcher.transfer(-1);
		launcher.outtake(0.8);
		packet.clearLines();
		packet.putAll(launcher.getPIDTelemetry(false));
		dash.sendTelemetryPacket(packet);
		telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
		while(setTarget[0] > drivetrain.getPosition()[0]){
			telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
			drivetrain.robotOrientedDrive(0.2, 0, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "first move loop"));

			packet.clearLines();
			sendTelemetry("grrr");
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
			telemetry.update();

		}
		drivetrain.robotOrientedDrive(0, 0, 0);
		launcher.fling(false);

		timer = new ElapsedTime();
		while(timer.seconds()<2){
			telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
			launcher.transfer(-1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first launch loop"));
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
			telemetry.update();
		}

		launcher.fling(true);

		timer = new ElapsedTime();
		while(timer.seconds()<1.2){
			launcher.transfer(-1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first wait loop"));
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			sendTelemetry("grrr");
			dash.sendTelemetryPacket(packet);
			telemetry.update();

		}

		launcher.fling(false);

		timer = new ElapsedTime();
		while(timer.seconds()<1.5){
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "second wait loop"));
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
			sendTelemetry("grrr");
			telemetry.update();

		}

		timer = new ElapsedTime();
		while(timer.seconds()<1){
			launcher.intake(1);
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			sendTelemetry("grrr");
			dash.sendTelemetryPacket(packet);
		}

		launcher.fling(true);

		timer = new ElapsedTime();
		while(timer.seconds()<2)
		{
			launcher.fling(true);
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			sendTelemetry("grrr");
			dash.sendTelemetryPacket(packet);
		}
		setTarget[0]=18;
		while(setTarget[0] > drivetrain.getPosition()[0]){
			launcher.outtake(0);
			drivetrain.robotOrientedDrive(.2, 0, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			sendTelemetry("grrr");
			dash.sendTelemetryPacket(packet);
		}

		drivetrain.robotOrientedDrive(0, 0, 0);
		packet.clearLines();
		packet.putAll(launcher.getPIDTelemetry(false));
		sendTelemetry("grrr");
		dash.sendTelemetryPacket(packet);
		telemetry.update();

			setTarget[2] = -111;

		while(setTarget[2] < drivetrain.getPosition()[2]){
			drivetrain.robotOrientedDrive(0, 0, -0.2);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			sendTelemetry("grrr");
			telemetry.update();
		}

		setTarget[1] = 29.5;
		while(setTarget[1] > drivetrain.getPosition()[1]) {
			drivetrain.robotOrientedDrive(0, 0.4, 0);
			drivetrain.updateOdo();
			sendTelemetry("grrr");
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}

		setTarget[0] = 28;
		while(setTarget[0] > drivetrain.getPosition()[0]){
			drivetrain.robotOrientedDrive(-0.25, 0, 0);
			launcher.intake(1);
			launcher.transfer(-1);
			sendTelemetry("grrr");
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

		setTarget[0] = 18;
		while(setTarget[0] < drivetrain.getPosition()[0]){
			launcher.fling(false);
			drivetrain.robotOrientedDrive(0.25, 0, 0);
			drivetrain.updateOdo();
			sendTelemetry("grrr");
			telemetry.update();
		}

		timer = new ElapsedTime();
		while(timer.seconds() < 0.5)
			drivetrain.robotOrientedDrive(0,0,0);

		setTarget[1] = 34;
		while(setTarget[1] < drivetrain.getPosition()[1]) {
			drivetrain.robotOrientedDrive(0, -0.4, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();

		}

		setTarget[2] = -5;
		while(setTarget[2] > drivetrain.getPosition()[2]){
			drivetrain.robotOrientedDrive(0, 0, 0.2);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}

		setTarget[0]= -8;
		while(setTarget[0] < drivetrain.getPosition()[0]){
			launcher.outtake(0.8);
			drivetrain.robotOrientedDrive(-.2, 0, 0);
			drivetrain.updateOdo();
			telemetry.update();
		}
		drivetrain.robotOrientedDrive(0, 0, 0);

		timer = new ElapsedTime();
		while(timer.seconds() < 2) {
			launcher.intake(1);
			launcher.transfer(-1);
			launcher.fling(true);
			sendTelemetry("Launch second collected ball");
		}

		while(true){
			sendTelemetry("waity waity waity");
			drivetrain.robotOrientedDrive(0,0,0);
			drivetrain.neutral();
			drivetrain.updateOdo();
		}
		/*requestOpModeStop();*/
	}
}