package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Path;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Path.MechanismNotFoundException;
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

		Path forward = new Path("movey movey");
		forward.queueStates(
				new ArrayList<Object[][]>() {{
					add(new Object[][]{{Drivetrain.class}, new Object[]{6, 6, 45}, new Object[]{0,0}});
				}}
		);

		hold(1000);

		Path sideways = new Path("Return");
		sideways.queueStates(
				new ArrayList<Object[][]>() {{
					add(new Object[][]{{Drivetrain.class}, new Object[]{(Runnable) () -> {
						try{
							Drivetrain dt = (Drivetrain) Path.getMechanism(Drivetrain.class);
							ElapsedTime timer = new ElapsedTime();
							while(timer.milliseconds() < 1000) {
								dt.robotOrientedDrive(0, 0, 0);
							}
						}catch(MechanismNotFoundException e) {
							telemetry.addLine(e.getMessage());
							telemetry.update();
							stop();
						}
					}}});
				}}
		);


	}
}