package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;


@Autonomous(name="RedClose", group="Autonomous")
public class RedClose extends OpMode {
	Drivetrain drivetrain;
	Launcher launcher;
	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		launcher = new Launcher(hardwareMap);
		telemetry.addLine((drivetrain.toString() + "Initializing"));
		telemetry.update();
	}

	@Override
	public void loop() {
		while(drivetrain.getPosition()[0] > -50) {
			drivetrain.robotOrientedDrive(-.3, 0, 0);
			drivetrain.updateOdo();
			launcher.outtake(0.8 );
			launcher.transfer(-1);

			telemetry.addLine((drivetrain.toString() + "first move loop"));
			telemetry.update();
		}

		drivetrain.robotOrientedDrive(0, 0, 0);
		ElapsedTime timer = new ElapsedTime();
		while(timer.seconds() <1) {
			drivetrain.robotOrientedDrive(0, 0, 0);
		}
		launcher.fling(true);
		while(timer.seconds() <2) {
			drivetrain.robotOrientedDrive(0,0,0);
		}
		timer = new ElapsedTime();
		while(timer.seconds() <6) {
			launcher.intake(1);
			launcher.transfer(-1);
		}




		requestOpModeStop();
	}
}