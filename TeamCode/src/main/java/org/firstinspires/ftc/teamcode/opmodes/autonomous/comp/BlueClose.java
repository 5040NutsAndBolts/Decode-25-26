package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.odo.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.ParentAuton;


@Autonomous(name="BlueClose", group="Autonomous")
public class BlueClose extends ParentAuton {
	Drivetrain drivetrain;

	@Override
	public void init() {
		drivetrain = new Drivetrain(hardwareMap);
		telemetry.addLine((drivetrain.toString() + "Initializing"));
		telemetry.update();
	}

	@Override
	public void loop() {
		while(drivetrain.getPosition()[0] < 24) {
			drivetrain.robotOrientedDrive(.2, 0, 0);
			drivetrain.updateOdo();
			telemetry.addLine((drivetrain.toString() + "first move loop"));
			telemetry.update();
		}
		requestOpModeStop();
	}
}