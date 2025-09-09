package org.firstinspires.ftc.teamcode.teleops;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

@TeleOp(name="JustDrivetrain", group="Teleop")
public class JustDrivetrain extends OpMode {
	private Drivetrain dt;

	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
	}

	@Override
	public void loop() {
		dt.robotOrientedDrive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
		telemetry.addLine(dt.toString());
		telemetry.update();
	}
}