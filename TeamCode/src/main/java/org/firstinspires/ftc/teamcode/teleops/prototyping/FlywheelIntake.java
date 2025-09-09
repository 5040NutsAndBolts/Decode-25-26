package org.firstinspires.ftc.teamcode.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="FlywheelOuttakePrototype", group="Teleop")
public class FlywheelIntake extends OpMode {
	private CRServo servo;

	@Override
	public void init() {
		servo = hardwareMap.get(CRServo.class, "motor");
	}

	@Override
	public void loop() {
		servo.setPower(gamepad1.left_stick_y);
		telemetry.addLine("Motor Power: " + servo.getPower());
		telemetry.update();
	}

}
