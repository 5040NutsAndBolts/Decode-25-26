package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.helpers.PID;
import java.util.HashMap;
import java.util.Map;

public class Launcher {
	public final DcMotorEx flywheel;
	private final DcMotorEx wheelMotor;
	public final DcMotorEx transfer;
	private final CRServo  flingServo;
	public Launcher(@NonNull HardwareMap hardwareMap) {
		//Motor initialization
		flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
		wheelMotor = hardwareMap.get(DcMotorEx.class, "Intake");
		wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		flingServo = hardwareMap.get(CRServo.class, "Fling");
		transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
		transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

	public double flywheelRPMS() {
		double currentTPS = flywheel.getVelocity();
		return (currentTPS * (2.67857485));
	}


	public void fling(boolean in) {
		flingServo.setPower(in ? 1 : 0);
	}
	public void fling(double in) {
		flingServo.setPower(in);
	}


	@NonNull
	@Override
	public String toString() {
		return
				"Flywheel RPMs: " + flywheelRPMS()+ "\n" +
				"Flywheel out power: " + flywheel.getPower() + "\n" +
				"Transfer servo power: " + transfer.getPower() + "\n" +
				"Wheel motor power: " + wheelMotor.getPower() + "\n" +
				"Flick: " + flingServo.getPower();
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