package org.firstinspires.ftc.teamcode.opmodes.teleops.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="SorterTest", group="Teleop")
public class Sorter extends OpMode {
	private DcMotor front,back;
	@Override
	public void init() {
		front = hardwareMap.get(DcMotor.class, "front");
		back = hardwareMap.get(DcMotor.class, "back");
		front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		back.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	@Override
	public void loop() {
		front.setPower(.35*gamepad1.left_stick_y);
		back.setPower(.35*gamepad1.left_stick_y);
	}
}
