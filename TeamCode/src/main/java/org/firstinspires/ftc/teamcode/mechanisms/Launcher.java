package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;

public class Launcher extends Mechanism {
	public boolean waitWorthy = false;
	private final DcMotorEx flywheel;
	private final DcMotorEx wheelMotor;
	//private final Servo flickServo;

	public Launcher(@NonNull HardwareMap hardwareMap) {
		//Motor initialization
		flywheel = hardwareMap.get(DcMotorEx.class, "fly");
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		wheelMotor = hardwareMap.get(DcMotorEx.class, "intake");
		wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//flickServo = hardwareMap.get(Servo.class, "Flick");
	}

	/**
	 * speed in double
	 * @param powers [flywheelOutSpeed, transferSpeed]
	 */
	@Override
	public void update(@NonNull Object[] powers) {
		assert powers.length == 2 || powers.length == 3;
		for(Object o : powers)
			assert o instanceof Double || o instanceof Float || o instanceof Integer;
		if (powers.length == 2) {
			//flick((Double) powers[0] == 0);
			flywheel.setPower((Double) powers[1]);
			wheelMotor.setPower((Double) powers[1]);
		}else {
			//flick((Double) powers[0] == 0);
			flywheel.setPower((Double) powers[1]);
			wheelMotor.setPower((Double) powers[2]);
		}
	}

	public void outtake(double power) {
		flywheel.setPower(power);
	}

	public void intake(double power) {
		wheelMotor.setPower(power);
	}

	//public void flick(boolean in) {
	//	if(in)
	//		flickServo.setPosition(1);
	//	else
	//		flickServo.setPosition(0);
	//}

	@Override
	protected boolean isFinished(@NonNull Object[] o) {
		return true;
		//assert o.length == 1 &&( o[0] instanceof Double || o[0] instanceof Float || o[0] instanceof Integer);
		//return flickServo.getPosition() == (double)o[0];
	}

	private long lastTime = 0;
	private int lastEnc;
	public float flywheelRPMS() {
		long cur = System.currentTimeMillis();
		long dTime = lastTime == 0 ? cur : (cur - lastTime);



	}

	@NonNull
	@Override
	public String toString() {
		return
				"Flywheel Out Power: " + flywheel.getPower() + "\n" +
				"Flywheel RPMs: " + flywheel.getPower() + "\n" +
				"Wheel Servo Power: " + wheelMotor.getPower();// + "\n" +
				//"Flick: " + flickServo.getPosition();
	}
}
