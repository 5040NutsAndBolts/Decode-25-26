package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Path;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="TestAuto", group="Autonomous")
public class TestAuto extends ParentAuton{
	Drivetrain drivetrain;
	private final PID xpid = new PID(.01,0.000001,0);
	private final PID ypid = new PID(.01,0.000001,0);
	private final PID rpid = new PID(0,0,0);

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
		xpid.setTarget(10);
		ypid.setTarget(10);
		rpid.setTarget(0);
		while(
				Math.abs(xpid.getTarget() - drivetrain.getPos()[0]) > .5 ||
				Math.abs(ypid.getTarget() - drivetrain.getPos()[1]) > .5 ||
				Math.abs(rpid.getTarget() - drivetrain.getPos()[2]) > 5
		) {
			drivetrain.updateOdo();
			drivetrain.fieldOrientedDrive(
					xpid.autoControl(drivetrain.getPos()[0]),
					ypid.autoControl(drivetrain.getPos()[1]),
					rpid.autoControl(drivetrain.getPos()[2])
			);
			telemetry.addLine(xpid.getTarget() + "\n" +(drivetrain.toString() + "\nLoop 1\nRuntime:" + getRuntime()));
			telemetry.update();
		}
		drivetrain.updateOdo();
		telemetry.addLine((drivetrain.toString() + "\nMethod\nRuntime:" + getRuntime()));
		telemetry.update();

		/*
		while(!drivetrain.isFinished(new double[] {.1,3})){
			drivetrain.update(new double[] {10,10,0});
			telemetry.addLine((drivetrain.toString() + "\nL1\nRuntime:" + getRuntime()));
			telemetry.update();
		}
		drivetrain.robotOrientedDrive(0,0,0);
		drivetrain.updateOdo();
		telemetry.addLine((drivetrain.toString() + "\nOL\nRuntime:" + getRuntime()));
		telemetry.update();*/
	}
}