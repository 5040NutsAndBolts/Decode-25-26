package org.firstinspires.ftc.teamcode.opmodes.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

@TeleOp(name="FlywheelIntakePrototype", group="Teleop")
public class FlywheelIntake extends OpMode {
	private DcMotor spin;
	private Drivetrain dt;

	@Override
	public void init() {
		spin = hardwareMap.get(DcMotor.class, "spin");
		dt = new Drivetrain(hardwareMap);
	}

	@Override
	public void loop() {
		spin.setPower(gamepad1.right_trigger);
		spin.setPower(-gamepad1.left_trigger);
		dt.robotOrientedDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);
		telemetry.addLine("spin Power: " + spin.getPower());
		telemetry.update();
	}
}
