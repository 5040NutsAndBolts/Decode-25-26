package org.firstinspires.ftc.teamcode.opmodes.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FlywheelOuttakePrototype", group="Teleop")
public class FlywheelOuttake extends OpMode {
	private DcMotor motor;
	private double speed = 0.5;
	@Override
	public void init() {
		motor = hardwareMap.get(DcMotor.class, "motor");
	}

	@Override
	public void loop() {
		
		if(gamepad1.a && speed <= 1)
			speed += 0.1;
		if(gamepad1.b && speed >= 0)
			speed -= 0.1;
		while(gamepad1.left_stick_y > 0.1)
			motor.setPower(speed);
		while(gamepad1.left_stick_y < -0.1)
			motor.setPower(-speed);
		
		telemetry.addLine("Motor Power: " + motor.getPower());
		telemetry.update();
	}
}
