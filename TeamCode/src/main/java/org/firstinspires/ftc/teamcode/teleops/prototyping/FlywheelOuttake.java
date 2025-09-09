package org.firstinspires.ftc.teamcode.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

import java.lang.annotation.Annotation;

@TeleOp(name="FlywheelOuttakePrototype", group="Teleop")
public class FlywheelOuttake extends OpMode {
	private DcMotor motor;

	@Override
	public void init() {
		motor = hardwareMap.get(DcMotor.class, "motor");
	}

	@Override
	public void loop() {
		motor.setPower(gamepad1.left_stick_y);
		telemetry.addLine("Motor Power: " + motor.getPower());
		telemetry.update();
	}

}
