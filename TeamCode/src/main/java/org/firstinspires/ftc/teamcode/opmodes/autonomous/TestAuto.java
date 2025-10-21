package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Path;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="TestAuto", group="Autonomous")
public class TestAuto extends ParentAuton{
	Drivetrain drivetrain;

	@Override
	public void init() {
		 drivetrain = new Drivetrain(hardwareMap);
		telemetry.addLine((drivetrain + "I"));
		telemetry.update();
	}

	@Override
	public void init_loop() {
		super.init_loop();
		telemetry.addLine((drivetrain.toString() + "IL"));
		telemetry.update();
	}

	@Override
	public void loop() {
		while(!drivetrain.isFinished(new double[] {.1,3})){
			drivetrain.update(new double[] {10,10,0});
			telemetry.addLine((drivetrain.toString() + "\nL1\nRuntime:" + getRuntime()));
			telemetry.update();
		}
		drivetrain.robotOrientedDrive(0,0,0);
		drivetrain.updateOdo();
		telemetry.addLine((drivetrain.toString() + "\nOL\nRuntime:" + getRuntime()));
		telemetry.update();
	}
}