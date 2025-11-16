package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.ParentAuton;


@Autonomous(name="RedFar", group="Autonomous")
public class RedFar extends ParentAuton {
	Drivetrain drivetrain;
	Odometry odo;
	Launcher launcher;
	double[] setTarget;
	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		launcher = new Launcher(hardwareMap);
		telemetry.addLine((drivetrain + "I"));
		drivetrain.updateOdo();
		odo = new Odometry(hardwareMap, 0, 0);
		telemetry.update();
		setTarget = new double[]{
				0, -7.75, 0
		};
	}
	@Override
	public void init_loop() {
		super.init_loop();

		telemetry.addLine((drivetrain.toString() + "IL"));
		telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
		telemetry.addLine(""+drivetrain.getPosition()[0]);
		telemetry.addLine(""+drivetrain.getPosition()[1]);
		odo.update();
		telemetry.update();
	}


	@Override
	public void loop() {
		launcher.transfer(1);
		launcher.outtake(0.82);
		telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
		while(setTarget[1] < drivetrain.getPosition()[1]){
			telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
			drivetrain.robotOrientedDrive(.2, 0, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "first move loop"));
			telemetry.update();
		}
		drivetrain.robotOrientedDrive(0, 0, 0);
		launcher.flick(false);
		launcher.outtake(1);

		ElapsedTime timer = new ElapsedTime();
		while(timer.seconds()<4){
			telemetry.addLine("Launcher RPMs: " + launcher.flywheelRPMS());
			launcher.transfer(1);
			launcher.transfer(1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first launch loop"));
			telemetry.update();
		}

		launcher.flick(true);

		timer = new ElapsedTime();
		while(timer.seconds()<2){
			launcher.transfer(1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first wait loop"));
			telemetry.update();
		}

		launcher.flick(false);

		timer = new ElapsedTime();
		while(timer.seconds()<1.5){
			launcher.transfer(1);
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "second wait loop"));
			telemetry.update();
		}

		setTarget[1]=-18;
		while(setTarget[1] < drivetrain.getPosition()[1]){
			launcher.transfer(1);
			drivetrain.robotOrientedDrive(.2, -.15, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}
		drivetrain.robotOrientedDrive(0, 0, 0);

		requestOpModeStop();
	}
}