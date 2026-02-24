package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name="MTPTest", group="Teleop")
public class MTPTest extends OpMode {
	private Drivetrain dt;
	FtcDashboard dash;
	TelemetryPacket packet;


	boolean notWithinXY (double[] target, double[] posMaxError, double[] velocityMax) {
		double[] currentPosition = dt.getPosition();
		double[] currentVelocity = dt.getVelocity();
		boolean within = true;

		for(int i = 0; i < 2; i++)
			if(Math.abs(target[i]-currentPosition[i]) > posMaxError[i] || Math.abs(currentVelocity[i]) > velocityMax[i])
				within = false;
		return !within;
	}

	boolean notWithinR (double target, double maxError, double velocityMax) {
		return !(Math.abs(target - dt.getPosition()[2]) < maxError && Math.abs(dt.getVelocity()[2]) < velocityMax);
	}

	void setR (double target, double maxError, double velocityMax) {
		PID rotationController = new PID(.0375, 3e-5, 0);
		rotationController.setTarget(target);
		while(notWithinR(target,maxError,velocityMax)) {
			dt.fieldOrientedDrive(0, 0, rotationController.autoControl(dt.getPosition()[2]));

			HashMap<String, Object> map = new HashMap<>();
			map.put("R PID Output", rotationController.getCurrentOutput());
			map.put("Current Error", target - dt.getPosition()[2]);
			map.put("Current", dt.getPosition()[2]);
			map.put("Target",target);
			map.put("R PID TOS", rotationController.toString());

			sendTelemetry("Setting rotation to: " + target, map);
		}
	}

	void setXY (double[] target, double[] maxError, double[] velocityMax) {
		PID xController = new PID(.032, 7e-6, 0);
		PID yController = new PID(.032, 7e-6, 0);
		xController.setTarget(target[0]);
		yController.setTarget(target[1]);
		while(notWithinXY(target,maxError,velocityMax)) {
			dt.fieldOrientedDrive(
					-xController.autoControl(dt.getPosition()[0])
					, -yController.autoControl(dt.getPosition()[1])
					, 0);
			HashMap<String, Object> map = new HashMap<>();
			map.put("Velocities", Arrays.toString(dt.getVelocity()));
			map.put("X Error", ((target[0] - dt.getPosition()[0])));
			map.put("Y Error", ((target[1] - dt.getPosition()[1])));
			map.put("DT TOS", dt.toString());
			map.put("XC TOS", xController.toString());
			map.put("YC TOS", yController.toString());
			sendTelemetry("Setting position to: " + Arrays.toString(target), map);
		}
	}

	private void sendTelemetry(String loopName, Map<String, Object> additional) {
		packet.clearLines();
		HashMap<String, Object> map = new HashMap<>(additional);
		map.put("Name", loopName);
		map.put("Pos", Arrays.toString(dt.getPosition()));
		map.put("RPMS: ",la.flywheelRPMS());
		map.put("RPM Target: ",fC.getTarget());
		packet.putAll(map);
		dash.sendTelemetryPacket(packet);
		packet.clearLines();
	}

	private void sendTelemetry(String loopName) {
		packet = new TelemetryPacket();
		HashMap<String, Object> map = new HashMap<>();
		map.put("Name", loopName);
		map.put("Pos", Arrays.toString(dt.getPosition()));
		map.put("RPMS: ",la.flywheelRPMS());
		map.put("RPM Target: ",fC.getTarget());
		packet.putAll(map);
		dash.sendTelemetryPacket(packet);
		packet.clearLines();
	}

	void moveTo(
			double[] targets,
			double[] maxErrors,
			double[] maxVelocities
	) {
		while(notWithinR(targets[2],maxErrors[2],maxVelocities[2]) || notWithinXY(new double[] {targets[0], targets[1]}, new double[] {maxErrors[0],maxErrors[1]}, new double[] {maxVelocities[0], maxVelocities[1]})){
			setXY(new double[] {targets[0], targets[1]}, new double[] {maxErrors[0],maxErrors[1]}, new double[] {maxVelocities[0], maxVelocities[1]});
			setR(targets[2], maxErrors[2], maxErrors[2]);
		}
		dt.robotOrientedDrive(0,0,0);
	}

	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
		while(packet == null)
			packet = new TelemetryPacket();
		dash = FtcDashboard.getInstance();
		dt = new Drivetrain(hardwareMap);
		la = new Launcher(hardwareMap);
		fC = new PID(.0075, 3e-8, 0, la::flywheelRPMS, 0);
		sendTelemetry("Initialization");
		dt.resetOdo();
	}
	Launcher la;

	PID fC;
	@Override
	public void loop() {
		//la.setOuttakePower(.65);
		moveTo(
				new double[] {6.25,6.25,30},
				new double[] {1.5,1.5,3},
				new double[] {3,3,6}
		);

		/*
		fC.setTarget(6000);
		while(la.flywheelRPMS() < fC.getTarget() *.975){
			la.setOuttakePower(fC.autoControl());
			la.fling(1);
			sendTelemetry("Spinning up launcher");
		}

		byte shotcount = 0;
		long lastShotTime = 0;
		while(shotcount < 3) {
			la.setOuttakePower(fC.autoControl());
			ElapsedTime e1 = new ElapsedTime();
			if(la.flywheelRPMS() > fC.getTarget() *.98    && System.currentTimeMillis() - lastShotTime > 1250){
				while(e1.seconds() < 1 + (shotcount == 2 ? 2 : 0)) {
					la.intake(-1);
					la.transfer(1);
					la.setOuttakePower(1);
					sendTelemetry("Attempting shot");
					shotcount++;
					lastShotTime = System.currentTimeMillis();
				}
			}else {
				la.transfer(0);
				la.intake(0);
				sendTelemetry("Spinning up, launch prepared");
			}
		}
		*/

		setXY(
			new double[] {24, 6.25},
			new double[] {2, 4},
			new double[] {100,100}
		);
		setR(-90,2,100);

		while(dt.getPosition()[1] < 40) {
			la.transfer(1);
			la.intake(-1);
			dt.robotOrientedDrive(.35,0,0);
			sendTelemetry("Picking up artifacts");
		}
		dt.robotOrientedDrive(0,0,0);

		setXY(
				new double[] {6.25,6.25},
				new double[] {1.5,1.5},
				new double[] {100,100}
		);
		setR(30,3,6);

		/*
		fC.setTarget(6000);
		while(la.flywheelRPMS() < fC.getTarget() *.975){
			la.setOuttakePower(fC.autoControl());
			la.fling(1);
			sendTelemetry("Spinning up launcher");
		}

		byte shotcount = 0;
		long lastShotTime = 0;
		while(shotcount < 3) {
			la.setOuttakePower(fC.autoControl());
			ElapsedTime e1 = new ElapsedTime();
			if(la.flywheelRPMS() > fC.getTarget() *.98    && System.currentTimeMillis() - lastShotTime > 1250){
				while(e1.seconds() < 1 + (shotcount == 2 ? 2 : 0)) {
					la.intake(-1);
					la.transfer(1);
					la.setOuttakePower(1);
					sendTelemetry("Attempting shot");
					shotcount++;
					lastShotTime = System.currentTimeMillis();
				}
			}else {
				la.transfer(0);
				la.intake(0);
				sendTelemetry("Spinning up, launch prepared");
			}
		}
		*/

		setXY(
				new double[] {48, 6.25},
				new double[] {2, 4},
				new double[] {100,100}
		);
		setR(-90,2,100);

		while(dt.getPosition()[1] < 40) {
			la.transfer(1);
			la.intake(-1);
			dt.robotOrientedDrive(.35,0,0);
			sendTelemetry("Picking up artifacts 2");
		}
		dt.robotOrientedDrive(0,0,0);

		setXY(
				new double[] {6.25,6.25},
				new double[] {1.5,1.5},
				new double[] {100,100}
		);
		setR(30,3,6);

		while(true) {
			sendTelemetry("DONE");
			la.setOuttakePower(0);
			dt.robotOrientedDrive(0,0,0);
			la.transfer(0);
			la.intake(0);
			dt.neutral();
			dt.updateOdo();
			requestOpModeStop();
		}
	}
}
