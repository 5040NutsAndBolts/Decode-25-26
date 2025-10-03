package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Path;

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
			telemetry.addLine(((timeout - timer.milliseconds()) / 1000.0) + "s left");
			telemetry.update();
		}
	}

	@Override
	public void init() {
		dashboard = FtcDashboard.getInstance().getTelemetry();
	}

	@Override
	public void init_loop() {
		telemetry.addLine("Initializing...");
		telemetry.update();
	}

	@Override
	public void loop() {
		for(Path p : paths) {
			while(!p.isFinished()) {
				p.update();
				telemetry.addLine(p.toString());
				telemetry.update();
			}
		}
	}
}