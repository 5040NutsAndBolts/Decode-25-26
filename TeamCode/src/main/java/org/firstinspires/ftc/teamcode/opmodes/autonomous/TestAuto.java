package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helpers.states.Mechanism;
import org.firstinspires.ftc.teamcode.helpers.states.Path;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

import java.util.ArrayList;

@Autonomous(name="TestAuto", group="Autonomous")
public class TestAuto extends ParentAuton{
	@Override
	public void init() {
		super.init();

		Path.addMechanisms(
				new ArrayList<Mechanism>() {{
					add(new Drivetrain(hardwareMap));
				}}
		);

		Path forward = new Path("Forward");
		forward.queueStates(
				new ArrayList<Object[][]>() {{
					add(new Object[][]{{Drivetrain.class}, new Object[]{0, 6, 0}});
				}}
		);
		hold(1000);
		Path sideways = new Path("Sideways and Turn");
		sideways.queueStates(
				new ArrayList<Object[][]>() {{
					add(new Object[][]{{Drivetrain.class}, new Object[]{6, 6, 90}});
				}}
		);
	}
}