package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

@TeleOp(name="FullTest", group="Teleop")
public class FullTest extends OpMode {
	Launcher la;
	Drivetrain dt;

	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
		la = new Launcher(hardwareMap);
	}

	@Override
	public void loop() {
		dt.robotOrientedDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
		la.intake(gamepad2.right_trigger);
		la.outtake(gamepad2.left_trigger);
		//la.flick(gamepad2.a);
		dt.toggleSlowMode(gamepad1.b);

		telemetry.addLine("Launcher: " + la.toString());
		telemetry.addLine("Drivetrain: " + dt.toString());
		telemetry.update();
	}
}
