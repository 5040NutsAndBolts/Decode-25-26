package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;

public class Launcher extends Mechanism {
	public boolean waitWorthy = true;
	private final DcMotorEx flywheel;
	private final DcMotorEx wheelMotor;
	private final Servo flickServo;
	private final CRServo transferServo;

	public Launcher(@NonNull HardwareMap hardwareMap) {
		//Motor initialization
		flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		wheelMotor = hardwareMap.get(DcMotorEx.class, "Intake");
		wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		flickServo = hardwareMap.get(Servo.class, "Flick");
		transferServo = hardwareMap.get(CRServo.class, "Conveyor");
	}

	/**
	 * speed in double
	 * @param powers [flywheelOutSpeed, transferSpeed]
	 */
	@Override
	public void update(@NonNull double[] powers) {
		assert powers.length == 2 || powers.length == 3;
		for(Object o : powers)
			assert o instanceof Double || o instanceof Float || o instanceof Integer;
		if (powers.length == 2) {
			flick((Double) powers[0] == 0);
			flywheel.setPower((Double) powers[1]);
			wheelMotor.setPower((Double) powers[1]);
		}else {
			flick((Double) powers[0] == 0);
			flywheel.setPower((Double) powers[1]);
			wheelMotor.setPower((Double) powers[2]);
		}
	}


	public void transfer(double power) {
		transferServo.setPower(power);
	}

	private final PID flywheelPID = new PID(.0065,0.000001,0);
	public void outtake(double power) {
		long farRPM = 5500;
		flywheelPID.setTarget(power > .5 ? farRPM : 0);
		double outpower = flywheelPID.autoControl(flywheelRPMS());
		flywheel.setPower(outpower > .2 ? outpower : 0);
	}

	public void intake(double power) {
		wheelMotor.setPower(power);
	}

	public void flick(boolean in) {
		if(in)
			flickServo.setPosition(1);
		else
			flickServo.setPosition(0);
	}

	@Override
	protected boolean isFinished(@NonNull double[] o) {
		return Math.abs(flywheelRPMS() - flywheelPID.getTarget()) < o[0];
	}

	public double flywheelRPMS() {
		double currentTPS = flywheel.getVelocity();
		double degreesPerTick = 2.67857485;
		return (currentTPS * degreesPerTick * 60);
	}

	@NonNull
	@Override
	public String toString() {
		return
				"Flywheel RPMs: " + flywheelRPMS()+ "\n" +
				"Flywheel out power: " + flywheel.getPower() + "\n" +
				"Transfer servo power: " + transferServo.getPower() + "\n" +
				"Wheel motor power: " + wheelMotor.getPower() + "\n" +
				"Flick: " + flickServo.getPosition() + "\n" +
				"Flywheel PID: " + flywheelPID;
	}
}