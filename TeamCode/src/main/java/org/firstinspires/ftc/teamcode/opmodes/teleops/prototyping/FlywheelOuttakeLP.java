package org.firstinspires.ftc.teamcode.opmodes.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FlywheelOuttakePrototypeLP", group="Teleop")
public class FlywheelOuttakeLP extends OpMode {
	private DcMotor motor;

	@Override
	public void init() {
		motor = hardwareMap.get(DcMotor.class, "motor");
	}

	@Override
	public void loop() {
		motor.setPower(.9);
		telemetry.addLine("Motor Power: " + motor.getPower());
		telemetry.update();
	}
}