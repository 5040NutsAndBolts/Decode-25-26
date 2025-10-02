package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.helpers.easypathing.Mechanism;

public class Launcher extends Mechanism {
	private final DcMotorEx flywheelOut, flywheelIn, transfer;

	public Launcher(@NonNull HardwareMap hardwareMap) {
		//Motor initialization
		flywheelIn = hardwareMap.get(DcMotorEx.class, "Fly In");
		flywheelOut = hardwareMap.get(DcMotorEx.class, "Fly Out");
		transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
	}

	/**
	 * speed in double
	 * @param powers [flywheelOutSpeed, flywheelInSpeed, transferSpeed]
	 */
	@Override
	public void update(@NonNull Object[] powers) {
		if(powers.length == 1) {
			assert powers[0] instanceof Double;
			flywheelOut.setPower((Double) powers[0]);
			flywheelIn.setPower((Double) powers[0]);
			transfer.setPower((Double) powers[0]);
		}else if(powers.length == 3) {
			for(Object o : powers)
				assert o instanceof Double;
			flywheelOut.setPower((Double) powers[0]);
			flywheelIn.setPower((Double) powers[1]);
			transfer.setPower((Double) powers[2]);
		}
		else throw new IllegalArgumentException("Powers must be either [speed] or [flywheelOutSpeed, flywheelInSpeed, transferSpeed]");
	}

	public void outtake(double power) {
		flywheelOut.setPower(power);
	}

	public void intake(double power) {
		flywheelIn.setPower(power);
		transfer.setPower(power);
	}

	@Override
	protected boolean isFinished(Object[] o) {
		return true;
	}

	@NonNull
	@Override
	public String toString() {
		return
				"Flywheel In Power: " + flywheelIn.getPower() + "\n" +
				"Flywheel Out Power: " + flywheelOut.getPower() + "\n" +
				"Transfer Power: " + transfer.getPower();
	}
}
