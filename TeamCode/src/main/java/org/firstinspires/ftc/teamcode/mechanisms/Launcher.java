package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;

public class Launcher extends Mechanism {
	public boolean waitWorthy = false;
	private final DcMotorEx flywheelOut, flywheelIn;
	private final CRServo wheelServo;
	private final Servo flickServo;

	public Launcher(@NonNull HardwareMap hardwareMap) {
		//Motor initialization
		flywheelIn = hardwareMap.get(DcMotorEx.class, "Fly In");
		flywheelOut = hardwareMap.get(DcMotorEx.class, "Fly Out");
		wheelServo = hardwareMap.get(CRServo.class, "Transfer");
		flickServo = hardwareMap.get(Servo.class, "Flick");
	}

	/**
	 * speed in double
	 * @param powers [flywheelOutSpeed, flywheelInSpeed, transferSpeed]
	 */
	@Override
	public void update(@NonNull Object[] powers) {
		assert powers.length == 2 || powers.length == 4;
		if (powers.length == 2) {
			for(Object o : powers)
				assert o instanceof Double || o instanceof Float || o instanceof Integer;
			flick((Double) powers[0] == 0);
			flywheelOut.setPower((Double) powers[1]);
			flywheelIn.setPower((Double) powers[1]);
			wheelServo.setPower((Double) powers[1]);
		}else {
			for(Object o : powers)
				assert o instanceof Double || o instanceof Float || o instanceof Integer;
			flick((Double) powers[0] == 0);
			flywheelOut.setPower((Double) powers[1]);
			flywheelIn.setPower((Double) powers[2]);
			wheelServo.setPower((Double) powers[3]);
		}
	}

	public void outtake(double power) {
		flywheelOut.setPower(power);
	}

	public void intake(double power) {
		flywheelIn.setPower(power);
		wheelServo.setPower(power);
	}

	public void flick(boolean in) {
		if(in)
			flickServo.setPosition(1);
		else
			flickServo.setPosition(0);
	}

	@Override
	protected boolean isFinished(@NonNull Object[] o) {
		assert o.length == 1 &&( o[0] instanceof Double || o[0] instanceof Float || o[0] instanceof Integer);
		return flickServo.getPosition() == (double)o[0];
	}

	@NonNull
	@Override
	public String toString() {
		return
				"Flywheel In Power: " + flywheelIn.getPower() + "\n" +
				"Flywheel Out Power: " + flywheelOut.getPower() + "\n" +
				"Wheel Servo Power: " + wheelServo.getPower();
	}
}
