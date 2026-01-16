package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
	private final DcMotorEx transfer;
	private final CRServo  flingServo;
	private final ColorSensor topColor, lowColor;

	public Launcher(@NonNull HardwareMap hardwareMap) {
		//Motor initialization
		flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		wheelMotor = hardwareMap.get(DcMotorEx.class, "Intake");
		wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		flingServo = hardwareMap.get(CRServo.class, "Fling");
		transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
		transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		topColor = hardwareMap.get(ColorSensor.class, "Top Color");
		lowColor = hardwareMap.get(ColorSensor.class, "Low Color");
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
			fling(powers[0] == 0);
			flywheel.setPower(powers[1]);
			wheelMotor.setPower(powers[1]);
		}else {
			fling(powers[0] == 0);
			flywheel.setPower(powers[1]);
			wheelMotor.setPower(powers[2]);
		}
	}

	public void transfer(double power) {
		transfer.setPower(power);
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

	public void fling(boolean in) {
		flingServo.setPower(in ? 1 : 0);
	}
	@Override
	protected boolean isFinished(@NonNull double[] o) {
		return Math.abs(flywheelRPMS() - flywheelPID.getTarget()) < o[0];
	}

	public double flywheelRPMS() {
		double currentTPS = flywheel.getVelocity();
		return (currentTPS * (2.67857485));
	}

	public Lights.Color topColor() {
		int gScore = (int) (topColor.green() - (topColor.blue()*.2));
		int oScore = topColor.red() + (int)(topColor.green()*.4);
		int pScore = topColor.blue() + (int) (topColor.red() * .7);
		int highest = Math.max(gScore, Math.max(oScore, pScore));
		if(highest < 300 || highest == oScore)
			return Lights.Color.BLOOD_ORANGE;
		else if (highest ==  pScore)
			return Lights.Color.PURPLE;
		else return Lights.Color.GREEN;
	}

	public Lights.Color lowColor() {
		int gScore = (int) (lowColor.green()-(lowColor.blue()*.2));
		int oScore = lowColor.red() + (int)(lowColor.green()*.4);
		int pScore = lowColor.blue() + (int) (lowColor.red() * .7);
		int highest = Math.max(gScore, Math.max(oScore, pScore));
		if(highest < 300 || highest == oScore)
			return Lights.Color.BLOOD_ORANGE;
		else if (highest ==  pScore)
			return Lights.Color.PURPLE;
		else return Lights.Color.GREEN;
	}

	@NonNull
	@Override
	public String toString() {
		return
				"topColor: r" + topColor.red() + " g" + topColor.green() + " b" + topColor.blue() + "\n" +
				"lowColor: r" + lowColor.red() + " g" + lowColor.green() + " b" + lowColor.blue() + "\n" +
				"Flywheel RPMs: " + flywheelRPMS()+ "\n" +
				"Flywheel out power: " + flywheel.getPower() + "\n" +
				"Transfer servo power: " + transfer.getPower() + "\n" +
				"Wheel motor power: " + wheelMotor.getPower() + "\n" +
				"Flick: " + flingServo.getPower() + "\n" +
				"Flywheel PID: " + flywheelPID + "\n";
	}

	public HashMap<String, Object> getPIDTelemetry(boolean inInit) {
		HashMap<String, Object> map = new HashMap<>();
		map.put("Flywheel PID Target", 5550);
		map.put("Flywheel Current", flywheelRPMS());
		map.put("Flywheel PID Output", flywheelPID.getCurrentOutput());
		//no idea why but this HAS to be calculated here
		double numero = nullifier ? ((5550 - flywheelRPMS()) * .0035) : .3;
		if (!inInit) flywheel.setPower(numero);
		else flywheel.setPower(0);
		map.put("Flywheel Real Input Power", numero);
		map.put("Flywheel Motor Power", flywheel.getPower());
		return map;
	}
}