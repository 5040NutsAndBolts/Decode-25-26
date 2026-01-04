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
	@Override
	public void init_loop() {
		super.init_loop();
		packet.clearLines();
		packet.putAll(launcher.getPIDTelemetry(true));
		dash.sendTelemetryPacket(packet);

		packet.putAll(aprilTag.getCameraTelemetry(true));


		telemetry.addLine((drivetrain.toString() + "IL"));
		telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
		telemetry.addLine(""+drivetrain.getPosition()[0]);
		telemetry.addLine(""+drivetrain.getPosition()[1]);
		telemetry.update();
	}

	@Override
	public void loop() {

		packet.putAll(aprilTag.getCameraTelemetry(true));

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
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
			telemetry.update();
		}
		drivetrain.robotOrientedDrive(0, 0, 0);
		launcher.fling(false);

		ElapsedTime timer = new ElapsedTime();
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
		while(timer.seconds()<2){
			launcher.transfer(-1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first wait loop"));
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
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
			telemetry.update();

		}

		timer = new ElapsedTime();
		while(timer.seconds()<1){
			launcher.intake(1);
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
		}

		launcher.fling(true);

		timer = new ElapsedTime();
		while(timer.seconds()<1.5)
		{
			launcher.fling(true);
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
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
			dash.sendTelemetryPacket(packet);
		}

			drivetrain.robotOrientedDrive(0, 0, 0);
		packet.clearLines();
		packet.putAll(launcher.getPIDTelemetry(false));
		dash.sendTelemetryPacket(packet);
		telemetry.update();

		timer = new ElapsedTime();
		while(timer.seconds() < 0.5)
			setTarget[2] = -111;

		while(setTarget[2] < drivetrain.getPosition()[2]){
			drivetrain.robotOrientedDrive(0, 0, -0.2);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}

		setTarget[1] = 26.7;
		while(setTarget[1] > drivetrain.getPosition()[1]) {
			drivetrain.robotOrientedDrive(0, 0.4, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}

		setTarget[0] = 28.7;
		while(setTarget[0] > drivetrain.getPosition()[0]){
			drivetrain.robotOrientedDrive(-0.25, 0, 0);
			launcher.intake(1);
			launcher.transfer(-1);
			launcher.outtake(0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}

		timer = new ElapsedTime();
		while(timer.seconds()<0.2)
			drivetrain.robotOrientedDrive(0,0,0);

		timer = new ElapsedTime();
		while(timer.seconds()<0.05)
			launcher.intake(0);
		//retrieve ball

		setTarget[0] = 21;
		while(setTarget[0] < drivetrain.getPosition()[0]){
			launcher.fling(false);
			drivetrain.robotOrientedDrive(0.25, 0, 0);
			drivetrain.updateOdo();
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

		setTarget[2] = 0;
		while(setTarget[2] > drivetrain.getPosition()[2]){
			drivetrain.robotOrientedDrive(0, 0, 0.2);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}

		setTarget[0]= 1.5;
		while(setTarget[0] < drivetrain.getPosition()[0]){
			launcher.outtake(0.8);
			drivetrain.robotOrientedDrive(-.2, 0, 0);
			drivetrain.updateOdo();
			telemetry.update();
		}

		timer = new ElapsedTime();
		while(timer.seconds() < 2.5) {

			drivetrain.robotOrientedDrive(0, 0, 0);
			packet.putAll(aprilTag.getCameraTelemetry(true));

		}

		timer = new ElapsedTime();
		while(timer.seconds() < 0.5)
			launcher.transfer(-1);

		launcher.fling(true);

		timer = new ElapsedTime();
		while(timer.seconds() < 2) {
			launcher.intake(1);
			launcher.transfer(-1);
		}
		timer = new ElapsedTime();
		while(timer.seconds() < 3)
			launcher.fling(true);


		//23.8 x
		requestOpModeStop();
	}
}