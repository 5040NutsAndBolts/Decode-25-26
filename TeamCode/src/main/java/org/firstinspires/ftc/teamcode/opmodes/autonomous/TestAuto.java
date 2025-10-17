package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Path;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="TestAuto", group="Autonomous")
public class TestAuto extends ParentAuton{
	private final ArrayList<Path> paths = new ArrayList<>();

	@Override
	public void init() {
		super.init();

		Drivetrain drivetrain = new Drivetrain(hardwareMap);
		Path forward = new Path("movey movey");
		forward.addMechanism(drivetrain);

		ArrayList<Object[]> states = new ArrayList<>();

		states.add(new Object[]{
				(Runnable) () -> {
					while (!drivetrain.isFinished(new Object[]{1.0, 1.0, 5.0})) {
						drivetrain.update(new Object[]{ 10.0, 10.0, 45.0 });
						telemetry.addLine("IN MOVEY MOVEY RUNNABLE");
						telemetry.update();
					}
				}
		});

		forward.queueStates(states);
		paths.add(forward);
	}

	private int iter = 0;
	@Override
	public void loop() {
		telemetry.addLine(Arrays.toString(paths.toArray()));
		telemetry.update();
		paths.get(0).update();
	}
}