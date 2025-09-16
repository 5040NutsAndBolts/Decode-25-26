package org.firstinspires.ftc.teamcode.opmodes.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FlywheelIntakePrototype", group="Teleop")
public class FlywheelIntake extends OpMode {
	private DcMotor rot, spin;

	@Override
	public void init() {
		rot = hardwareMap.get(DcMotor.class, "rot");
		spin = hardwareMap.get(DcMotor.class, "spin");
	}

	@Override
	public void loop() {
		rot.setPower(gamepad1.left_stick_y);
		spin.setPower(gamepad1.right_stick_x);
		telemetry.addLine("spin Power: " + spin.getPower());
		telemetry.addLine("rot Power: " + rot.getPower());
		telemetry.update();
	}
}
