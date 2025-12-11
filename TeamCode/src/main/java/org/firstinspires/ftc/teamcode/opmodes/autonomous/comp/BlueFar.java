package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.ParentAuton;


@Autonomous(name="BlueFar", group="Autonomous")
public class BlueFar extends ParentAuton {
	Drivetrain drivetrain;
	Launcher launcher;
	double[] setTarget;
	TelemetryPacket packet;
	FtcDashboard dash;
	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		launcher = new Launcher(hardwareMap);
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

		telemetry.addLine((drivetrain.toString() + "IL"));
		telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
		telemetry.addLine(""+drivetrain.getPosition()[0]);
		telemetry.addLine(""+drivetrain.getPosition()[1]);
		telemetry.update();
	}

	@Override
	public void loop() {
		drivetrain.resetOdo();
		launcher.transfer(1);
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
		launcher.flick(false);

		ElapsedTime timer = new ElapsedTime();
		while(timer.seconds()<4){
			telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
			launcher.transfer(1);
			launcher.transfer(1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first launch loop"));
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
			telemetry.update();
		}

		launcher.flick(true);

		timer = new ElapsedTime();
		while(timer.seconds()<2){
			launcher.transfer(1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first wait loop"));
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
			telemetry.update();
		}

		launcher.flick(false);

		timer = new ElapsedTime();
		while(timer.seconds()<1.5){
			launcher.transfer(1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "second wait loop"));
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
			telemetry.update();
		}

		timer = new ElapsedTime();
		while(timer.seconds()<5){
			launcher.intake(1);
			launcher.transfer(1);
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
		}

		launcher.flick(true);

		timer = new ElapsedTime();
		while(timer.seconds()<3)
		{
			launcher.flick(true);
			packet.clearLines();
			packet.putAll(launcher.getPIDTelemetry(false));
			dash.sendTelemetryPacket(packet);
		}
		setTarget[0]=18;
		while(setTarget[0] > drivetrain.getPosition()[0]){
			launcher.transfer(1);
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

		requestOpModeStop();
	}
}