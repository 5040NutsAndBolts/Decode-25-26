package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;
import java.util.HashMap;
import java.util.Map;

public class Launcher extends Mechanism {
	public boolean waitWorthy = true;
	private final DcMotorEx flywheel;
	private final DcMotorEx wheelMotor;
	private final Servo flickServo;
	private final CRServo transferServo;

	public Launcher(@NonNull HardwareMap hardwareMap) {
		//Motor initialization
		flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
			flick(powers[0] == 0);
			flywheel.setPower(powers[1]);
			wheelMotor.setPower(powers[1]);
		}else {
			flick(powers[0] == 0);
			flywheel.setPower(powers[1]);
			wheelMotor.setPower(powers[2]);
		}
	}

	public void transfer(double power) {
		transferServo.setPower(power);
	}

	private boolean nullifier = true;
	private final PID flywheelPID = new PID(.03,0.0001,0.0, this::flywheelRPMS, .5);
	public void outtake(double power) {
		nullifier = power > .5;
		getPIDTelemetry(false);
	}

	public void intake(double power) {
		wheelMotor.setPower(power);
	}
	public void setOuttakePower(double p) {
		flywheel.setPower(p);
	}

	public void flick(boolean in) {
		flickServo.setPosition(in ? 1 : 0);
	}
	boolean lastFlickIn = false;
	public void setFlick(boolean in) {
		flickServo.setPosition(in && lastFlickIn ? flickServo.getPosition() == 1 ? 0 : 1 : 0);
		lastFlickIn = in;
	}
	@Override
	protected boolean isFinished(@NonNull double[] o) {
		return Math.abs(flywheelRPMS() - flywheelPID.getTarget()) < o[0];
	}

	public double flywheelRPMS() {
		double currentTPS = flywheel.getVelocity();
		return (currentTPS * (2.67857485));
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
				"Flywheel PID: " + flywheelPID + "\n";

	}

	public Map<String, Object> getPIDTelemetry(boolean inInit) {
		HashMap<String, Object> map = new HashMap<>();
		map.put("Flywheel PID Target", 5100);
		map.put("Flywheel Current", flywheelRPMS());
		map.put("Flywheel PID Output", flywheelPID.getCurrentOutput());
		//no idea why but this HAS to be calculated here
		double numero = nullifier ? ((5100 - flywheelRPMS()) * .0035) : .3;
		if (!inInit) flywheel.setPower(numero);
		map.put("Flywheel Real Input Power", numero);
		map.put("Flywheel Motor Power", flywheel.getPower());
		return map;
	}
}