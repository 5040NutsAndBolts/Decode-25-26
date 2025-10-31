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
				5, 0, 0
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
			ElapsedTime timer = new ElapsedTime();
			while(timer.seconds()<2){
				launcher.outtake(1.0);
			}
			launcher.flick(true);
			while(timer.seconds()<2){}
			launcher.flick(false);
			while(setTarget != drivetrain.getPosition()){
				drivetrain.robotOrientedDrive(0.2, 0, 0);
				drivetrain.updateOdo();
				telemetry.addLine((drivetrain.toString() + "IL"));
				telemetry.update();
			}
		}

	}
