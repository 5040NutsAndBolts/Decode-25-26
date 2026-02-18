package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

import java.util.HashMap;

@TeleOp(name="LauncherTest", group="Teleop")
public class LauncherTest extends OpMode {
	private Launcher la;
	FtcDashboard dash;
	TelemetryPacket packet;

	@Override
	public void init() {
		dash = FtcDashboard.getInstance();
		packet=new TelemetryPacket();
		la = new Launcher(hardwareMap);
		sendTelemetry();
	}


	PID pid = new PID(.006, 1e-8, 0);

	private void sendTelemetry () {
		packet.clearLines();
		HashMap<String, Object> map = new HashMap<>();
		map.put("RPM",la.flywheelRPMS());
		map.put("Target", 5300);
		map.put("Power",la.flywheel.getPower());
		packet.addLine(pid.toString());
		packet.putAll(map);
		dash.sendTelemetryPacket(packet);
	}

	@Override
	public void loop() {
		la.intake(gamepad1.left_trigger-gamepad1.right_trigger);
		la.transfer(-gamepad2.left_stick_y);
		la.fling(gamepad2.b);
		pid.setTarget(5300);
		la.setOuttakePower(pid.autoControl(la.flywheelRPMS()));
		if(Math.abs(la.flywheelRPMS() - 5300 ) < 100) {
			gamepad1.rumble(100);
			gamepad2.rumble(100);
		}

		sendTelemetry();
		telemetry.addLine("Launcher: \n" + la.toString());
		telemetry.update();
	}
}
