package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.states.Path;
import java.util.ArrayList;

@Disabled
public class ParentAuton extends OpMode {
	protected ArrayList<Path> paths = new ArrayList<>();
	protected Telemetry dashboard;

	/**
	 * holds the robot for a certain amount of time
	 * @param timeout time in milliseconds
	 */
	protected void hold(long timeout) {
		ElapsedTime timer = new ElapsedTime();
		while (timer.milliseconds() < timeout) {
			telemetry.addLine(((timeout-timer.milliseconds()) / 1000.0)+"s left");
			telemetry.update();
		}
	}

	@Override
	public void init() {
		dashboard = FtcDashboard.getInstance().getTelemetry();
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
		for(Path p : paths)
			while(!p.isFinished()) {
				p.update();
				telemetry.addLine("Runtime: "+(time/1000));
				telemetry.addLine(p.toString());
				telemetry.update();

				dashboard.addData("Runtime", time/1000);
				dashboard.addData("Path", p.toString());
				dashboard.update();
			}
	}
}
