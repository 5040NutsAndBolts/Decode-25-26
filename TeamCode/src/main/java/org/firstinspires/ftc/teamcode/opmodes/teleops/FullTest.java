package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Lights;

@TeleOp(name="FullTest", group="Teleop")
public class FullTest extends OpMode {
	Launcher la;
	Drivetrain dt;

	Lights lH, lL;

	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
		la = new Launcher(hardwareMap);
		lH = new Lights(hardwareMap, "Lights High");
		lL = new Lights(hardwareMap, "Lights Low");
	}

	@Override
	public void loop() {
		dt.robotOrientedDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
		dt.updateOdo();

		la.intake(gamepad1.right_trigger > 0.2 ? -1 : 0);
		if( gamepad1.left_trigger > .2 || gamepad1.right_trigger > .2 )
			la.intake(gamepad1.left_trigger-gamepad1.right_trigger);

		la.intake(gamepad2.right_trigger > 0.2 ? 1 : 0);

		la.transfer(gamepad2.left_stick_y);

		la.fling(gamepad2.b);

		dt.toggleSlowMode(gamepad1.b);

		la.setOuttakePower(gamepad2.left_trigger > 0.25 ? 0.89 : .2);

		la.fling(gamepad2.a);

		dt.toggleSlowMode(gamepad1.dpad_down);


		if(la.flywheelRPMS() > 5100 && gamepad2.left_trigger > .15) {
			gamepad2.rumble(200);
			gamepad1.rumble(200);
			telemetry.addLine("rumbling, far");
		} else {
			lH.setPattern(la.topColor());
			lL.setPattern(la.lowColor());
		}

		telemetry.addLine("Launcher: \n" + la.toString());
		telemetry.addLine("Drivetrain: \n" + dt.toString());
		telemetry.update();
	}
}