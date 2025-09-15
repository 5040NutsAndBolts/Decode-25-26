package org.firstinspires.ftc.teamcode.opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.helpers.states.Path;
import java.util.ArrayList;

@Disabled
public class Autonomous extends OpMode {
	public ArrayList<Path> paths;

	public void update() {
		for(Path p : paths)
			while(!p.isFinished()) {
				p.update();
				telemetry.addLine("Runtime: "+time);
				telemetry.addLine(p.toString());
				telemetry.update();
			}
	}

	@Override
	public void init() {

	}

	@Override
	public void loop() {
		/* Example !!
		Path p = new Path();
		paths.add(p);
		p.queueStates(
				new ArrayList<Object[][]>() {{
					add(new Object[][]{{Drivetrain.class}, new Object[]{0, 0, 0}});
				}}
		);
		*/
	}
}
