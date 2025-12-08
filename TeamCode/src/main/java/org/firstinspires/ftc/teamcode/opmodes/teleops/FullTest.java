package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

@TeleOp(name="FullTest", group="Teleop")
public class FullTest extends OpMode {
	Launcher la;
	Drivetrain dt;

	double power = 0.9;

	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
		la = new Launcher(hardwareMap);
	}

	@Override
	public void loop() {
		dt.robotOrientedDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
		dt.updateOdo();

		la.intake(gamepad1.right_trigger > 0.2 ? -1 : 0);
		if( gamepad1.left_trigger > .2 || gamepad1.right_trigger > .2 )
			la.intake(gamepad1.left_trigger-gamepad1.right_trigger);

		la.intake(gamepad1.left_trigger > 0.2 ? 1 : 0);

		la.transfer(gamepad2.left_stick_y);

		la.flick(gamepad2.b);

		dt.toggleSlowMode(gamepad1.b);

/*		if(gamepad2.y && power > 0)
			power -= 0.05;
		if(gamepad2.x && power < 1)
			power += 0.05;

		la.outtake(gamepad2.left_trigger > .15 ? power : .2);
*/

		la.outtake(gamepad2.left_trigger > 0.5 ? 1 : 0);

		la.flick(gamepad2.a);

		dt.toggleSlowMode(gamepad1.dpad_down);

		if(la.flywheelRPMS() > 5750 && gamepad2.left_trigger > .15) {
			gamepad2.rumble(100);
			gamepad1.rumble(100);
			telemetry.addLine("rumbling");
			//li.setPattern(Lights.Color.GREEN);
		}//else {
			//li.setPattern(Lights.Color.BLOOD_ORANGE);
		//}

		telemetry.addLine("Flywheel Power: " + power);
		telemetry.addLine("Flick In: " + gamepad2.a);

		telemetry.addLine("Launcher: \n" + la.toString());
		telemetry.addLine("Drivetrain: \n" + dt.toString());
		telemetry.update();
	}
}