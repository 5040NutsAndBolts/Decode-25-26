package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;


@Autonomous(name="TestAuto", group="Autonomous")
public class TestAuto extends ParentAuton{
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
		telemetry.addLine(""+drivetrain.getPosition()[0]);
		telemetry.addLine(""+drivetrain.getPosition()[1]);
		odo.update();
		telemetry.update();
	}


	@Override
	public void loop() {
		while(setTarget[1] < drivetrain.getPosition()[1]){
			drivetrain.robotOrientedDrive(.2, 0, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "first move loop"));
			telemetry.update();
		}
		drivetrain.robotOrientedDrive(0, 0, 0);

		ElapsedTime timer = new ElapsedTime();
		while(timer.seconds()<2){
			drivetrain.robotOrientedDrive(0, 0, 0);
			launcher.outtake(1);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first launch loop"));
			telemetry.update();
		}

		launcher.flick(true);

		timer = new ElapsedTime();
		while(timer.seconds()<2){
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "first wait loop"));
			telemetry.update();
		}

		launcher.flick(false);

		timer = new ElapsedTime();
		while(timer.seconds()<1.5){
			drivetrain.robotOrientedDrive(0, 0, 0);
			telemetry.addLine(String.valueOf(timer.seconds()));
			telemetry.addLine((drivetrain.toString() + "second wait loop"));
			telemetry.update();
		}

		setTarget[1]=-18;
		while(setTarget[1] < drivetrain.getPosition()[1]){
			drivetrain.robotOrientedDrive(.2, -.15, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "second move loop"));
			telemetry.update();
		}
		drivetrain.robotOrientedDrive(0, 0, 0);

		stop();
	}
}