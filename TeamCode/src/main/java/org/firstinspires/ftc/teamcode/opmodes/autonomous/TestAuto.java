package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Path;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import java.util.ArrayList;

@TeleOp(name="TestAuto", group="Teleop")
public class TestAuto extends ParentAuton{
	@Override
	public void init() {
		super.init();

		Path forward = new Path("movey movey");
		forward.queueStates(
				new ArrayList<Object[]>() {{
					add(new Object[]{
							Drivetrain.class,
							new Object[]{6.0, 6.0, 45.0},
							new Object[]{0.0,0.0}});
				}}
		);
		paths.add(forward);

		paths.forEach((p) -> p.addMechanism(new Drivetrain(hardwareMap)));
	}

	@Override
	public void loop() {
		super.loop();
	}
}