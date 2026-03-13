package org.firstinspires.ftc.teamcode.opmodes.autonomous.comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "Blue Far", group = "Autonomous")
public class BlueFar extends OpMode {
	private Drivetrain dt;
	FtcDashboard dash;
	TelemetryPacket packet;
	Launcher la;
	PID fC;

	/**
	 * Returns true if the robot is NOT yet within the position and velocity
	 * tolerances for X and Y. Both conditions must be satisfied simultaneously:
	 * position error must be small AND the robot must have slowed down.
	 */
	boolean notWithinXY(double[] target, double[] posMaxError, double[] velocityMax) {
		double[] currentPosition = dt.getPosition();
		double[] currentVelocity = dt.getVelocity();
		boolean within = true;

		for (int i = 0; i < 2; i++)
			if (Math.abs(target[i] - currentPosition[i]) > posMaxError[i]
					|| Math.abs(currentVelocity[i]) > velocityMax[i])
				within = false;

		return !within;
	}

	/**
	 * Returns true if the robot is NOT yet within the rotational tolerance.
	 * Uses angleDifference() so the error is always the shortest path,
	 * preventing the PID from spinning 270° when 90° would suffice.
	 */
	boolean notWithinR(double target, double maxError, double velocityMax) {
		double angularError = Drivetrain.angleDifference(target, dt.getPosition()[2]);
		return !(Math.abs(angularError) < maxError
				&& Math.abs(dt.getVelocity()[2]) < velocityMax);
	}

	/**
	 * Rotates the robot to a target heading in degrees.
	 *
	 * The PID target is set to 0 (zero error), and we feed it the
	 * shortest-path angular error directly. This means the PID always
	 * sees a value between -180 and 180 regardless of how many full
	 * rotations the robot has accumulated — solving the wraparound bug.
	 */
	void setR(double target, double maxError, double velocityMax) {
		PID rotationController = new PID(.0375, 3e-5, 0);
		rotationController.setTarget(0); // target is always "zero error"
		rotationController.setIntegralLimit(1e4);

		while (notWithinR(target, maxError, velocityMax)) {
			// Shortest-path error: always in (-180, 180], never wraps
			double error = Drivetrain.angleDifference(target, dt.getPosition()[2]);

			// Negate error so a positive error (need to turn CCW) produces
			// a positive rotation command
			dt.fieldOrientedDrive(0, 0, rotationController.autoControl(-error));

			HashMap<String, Object> map = new HashMap<>();
			map.put("R PID Output", rotationController.getCurrentOutput());
			map.put("Current Error", error);
			map.put("Current Heading", dt.getPosition()[2]);
			map.put("Target", target);
			map.put("R PID TOS", rotationController.toString());
			sendTelemetry("Setting rotation to: " + target, map);
		}
	}

	/**
	 * Moves the robot to a target XY position.
	 *
	 * PID controllers are created once per call, not once per loop tick,
	 * so the integral term accumulates meaningfully across the whole movement.
	 * Integral limits prevent windup/NaN.
	 */
	void setXY(double[] target, double[] maxError, double[] velocityMax) {
		// Created once here — not inside the loop — so integral history persists
		PID xController = new PID(.04, 1.4e-5, 0);
		PID yController = new PID(.04, 1.4e-5, 0);
		xController.setTarget(target[0]);
		yController.setTarget(target[1]);
		xController.setIntegralLimit(1e4); // prevents integral windup → NaN
		yController.setIntegralLimit(1e4);

		while (notWithinXY(target, maxError, velocityMax)) {
			double xOut = xController.autoControl(dt.getPosition()[0]);
			double yOut = yController.autoControl(dt.getPosition()[1]);

			// Guard: if PID somehow still produces NaN, zero the output
			// rather than sending NaN to the motors
			if (Double.isNaN(xOut) || Double.isInfinite(xOut)) xOut = 0;
			if (Double.isNaN(yOut) || Double.isInfinite(yOut)) yOut = 0;

			dt.fieldOrientedDrive(-xOut, -yOut, 0);

			HashMap<String, Object> map = new HashMap<>();
			map.put("Velocities", Arrays.toString(dt.getVelocity()));
			map.put("X Error", target[0] - dt.getPosition()[0]);
			map.put("Y Error", target[1] - dt.getPosition()[1]);
			map.put("DT TOS", dt.toString());
			map.put("XC TOS", xController.toString());
			map.put("YC TOS", yController.toString());
			sendTelemetry("Setting position to: " + Arrays.toString(target), map);
		}
	}

	/**
	 * Moves the robot to a full [x, y, heading] pose.
	 * XY and rotation are run sequentially: XY first, then rotation.
	 * The outer loop re-checks both in case rotation disturbed XY or vice versa.
	 *
	 * Note: PID controllers are intentionally inside setXY/setR so they reset
	 * cleanly for each segment — the outer loop here is just a convergence check.
	 */
	void moveTo(double[] targets, double[] maxErrors, double[] maxVelocities) {
		while (notWithinR(targets[2], maxErrors[2], maxVelocities[2])
				|| notWithinXY(
				new double[]{targets[0], targets[1]},
				new double[]{maxErrors[0], maxErrors[1]},
				new double[]{maxVelocities[0], maxVelocities[1]})) {

			setXY(new double[]{targets[0], targets[1]},
					new double[]{maxErrors[0], maxErrors[1]},
					new double[]{maxVelocities[0], maxVelocities[1]});

			setR(targets[2], maxErrors[2], maxVelocities[2]); // was passing maxErrors[2] twice before
		}
		dt.robotOrientedDrive(0, 0, 0);
	}

	private void sendTelemetry(String loopName, Map<String, Object> additional) {
		packet.clearLines();
		HashMap<String, Object> map = new HashMap<>(additional);
		map.put("Name", loopName);
		map.put("Pos", Arrays.toString(dt.getPosition()));
		map.put("Vel", Arrays.toString(dt.getVelocity()));
		map.put("RPMS: ", la.flywheelRPMS());
		map.put("RPM Target: ", fC.getTarget());
		packet.putAll(map);
		dash.sendTelemetryPacket(packet);
		packet.clearLines();
	}

	private void sendTelemetry(String loopName) {
		packet = new TelemetryPacket();
		HashMap<String, Object> map = new HashMap<>();
		map.put("Name", loopName);
		map.put("Pos", Arrays.toString(dt.getPosition()));
		map.put("Vel", Arrays.toString(dt.getVelocity()));
		map.put("RPMS: ", la.flywheelRPMS());
		map.put("RPM Target: ", fC.getTarget());
		packet.putAll(map);
		dash.sendTelemetryPacket(packet);
		packet.clearLines();
	}

	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
		la = new Launcher(hardwareMap);
		fC = new PID(.0075, 3e-8, 0, la::flywheelRPMS, 0);

		packet = new TelemetryPacket();
		dash = FtcDashboard.getInstance();

		sendTelemetry("Initialization");
		dt.resetOdo();
	}

	@Override
	public void loop() {
		la.outtake(.8);
		moveTo(
				new double[]{6.25, 6.25, 24},
				new double[]{2, 2, 2},
				new double[]{30, 30, 999 }
		);
		fC.setTarget(4300);
		while(la.flywheelRPMS() < fC.getTarget() *.98){
			la.setOuttakePower(fC.autoControl());
			la.gate(1);
			sendTelemetry("Spinning up launcher");
		}

		byte shotcount = 0;
		long lastShotTime = 0;
		while(shotcount < 3) {
			la.setOuttakePower(fC.autoControl());
			if(la.flywheelRPMS() > fC.getTarget() *.98    && System.currentTimeMillis() - lastShotTime > 1750){
				ElapsedTime e1 = new ElapsedTime();
				boolean first = true;
				while(e1.seconds() < .75 + (shotcount == 2 ? 1 : 0)) {
					la.intake(-1);
					la.transfer(1);
					la.setOuttakePower(1);
					sendTelemetry("Attempting shot "+ shotcount);
					if (first)shotcount++;
					first = false;
					lastShotTime = System.currentTimeMillis();
				}
				setR(24,2,999);
				dt.robotOrientedDrive(0,0,0);
			}else {
				la.transfer(0);
				la.intake(0);
				sendTelemetry("Spinning up, launch prepared");
			}
		}
		la.transfer(0);
		la.intake(0);
		la.outtake(.8);

		setXY(new double[]{21, 6.25}, new double[]{1.25, 5},   new double[]{100, 100});
		setR(-90, 3, 100);

		ElapsedTime k = new ElapsedTime();
		while (dt.getPosition()[1] < 47 && k.seconds() < 2) {
			la.gate(-1);
			la.transfer(1);
			la.intake(-1);
			dt.robotOrientedDrive(.3, 0, 0);
			sendTelemetry("Picking up artifacts");
		}
		la.gate(0);
		la.transfer(0);
		la.intake(0);
		dt.robotOrientedDrive(0, 0, 0);
		telemetry.update();

		setR(30, 29, 9999);
		moveTo(
				new double[]{20, 20, 24},
				new double[]{6, 6, 5},
				new double[]{100, 100, 1000}
		);
		setXY(
				new double[]{1.25, 1.25},
				new double[]{2, 2},
				new double[]{100, 100}
		);
		setR(24,2,3000);


		fC.setTarget(4300);
		long starttime = System.currentTimeMillis();
		while(la.flywheelRPMS() < fC.getTarget() *.975){
			la.setOuttakePower(fC.autoControl());
			la.transfer(0);
			la.intake(0);
			la.gate(1);
			sendTelemetry("Spinning up launcher");
			if(starttime < 500)
				la.gate(-1);
		}


		shotcount = 0;
		lastShotTime = 0;
		while(shotcount < 3) {
			la.setOuttakePower(fC.autoControl());
			if(la.flywheelRPMS() > fC.getTarget() *.98 && System.currentTimeMillis() - lastShotTime > 1750){
				ElapsedTime e1 = new ElapsedTime();
				boolean first = true;
				while(e1.seconds() < .75 + (shotcount == 2 ? 1 : 0)) {
					la.intake(-1);
					la.transfer(1);
					la.setOuttakePower(1);
					sendTelemetry("Attempting shot "+ shotcount);
					if (first)shotcount++;
					first = false;
					lastShotTime = System.currentTimeMillis();
				}
				setR(24,2,3000);
				dt.robotOrientedDrive(0,0,0);
			}else {
				la.transfer(0);
				la.intake(0);
				sendTelemetry("Spinning up, launch prepared");
			}
		}

		ElapsedTime e = new ElapsedTime();
		while(e.seconds() < 1)
			dt.robotOrientedDrive(-1,0,0);

		while (true) {
			sendTelemetry("DONE");
			la.setOuttakePower(0);
			dt.robotOrientedDrive(0, 0, 0);
			la.transfer(0);
			la.intake(0);
			la.outtake(0);
			dt.neutral();
			dt.updateOdo();
			requestOpModeStop();
		}
	}
}