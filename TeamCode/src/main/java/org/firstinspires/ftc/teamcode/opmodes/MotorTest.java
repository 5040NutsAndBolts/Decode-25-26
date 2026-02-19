package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorTest", group = "Teleop")
public class MotorTest extends OpMode {
	DcMotorEx motor;
	@Override
	public void init() {
		motor = hardwareMap.get(DcMotorEx.class, "Motor");
	}

	@Override
	public void loop() {
		motor.setPower(1);
	}
}
