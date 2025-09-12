package org.firstinspires.ftc.teamcode.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="FlywheelIntakePrototype", group="Teleop")
public class FlywheelIntake extends OpMode {
	private CRServo servo;

	@Override
	public void init() {
		servo = hardwareMap.get(CRServo.class, "servo");
	}

	@Override
	public void loop() {
		servo.setPower(gamepad1.left_stick_y);
		telemetry.addLine("Servo Power: " + servo.getPower());
		telemetry.update();
	}
}
