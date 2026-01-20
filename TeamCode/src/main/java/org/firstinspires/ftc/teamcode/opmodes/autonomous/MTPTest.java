package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

import java.util.HashMap;

public class MTPTest extends OpMode {
	private Drivetrain dt;
	FtcDashboard dash;
	TelemetryPacket packet;


	private boolean notWithin(double[] target, double[] positionTolerances, double[] velocityTolerances) {
		double[] currentPos = dt.getPosition();
		double[] currentVel = dt.getVelocity();

		boolean isWithin = true;

		//Check position tolerances
		for(int i = 0; i < 3; i++)
			if(Math.abs(target[i] - currentPos[i]) > positionTolerances[i])
				isWithin = false;

		//Check velocity tolerances
		for(int i = 0; i < 3; i++)
			if(Math.abs(currentVel[i] - target[i]) > velocityTolerances[i])
				isWithin = false;

		return isWithin;
	}


	private boolean notWithin(double[] target) {
		double[] currentPos = dt.getPosition();
		double[] currentVel = dt.getVelocity();

		boolean isWithin = true;

		//Check position tolerances
		for(int i = 0; i < 3; i++)
			if(Math.abs(target[i] - currentPos[i]) > 1)
				isWithin = false;

		//Check velocity tolerances
		for(int i = 0; i < 3; i++)
			if(Math.abs(currentVel[i] - target[i]) >( i == 2 ? 3 : 1))
				isWithin = false;

		return isWithin;
	}

	private void sendTelemetry(String loopName) {
		dt.updateOdo();
		packet.clearLines();
		HashMap<String, Object> map = new HashMap<>(dt.getPIDTelemetry());
		map.put("Name", loopName);
		packet.putAll(map);
		dash.sendTelemetryPacket(packet);
		dash=FtcDashboard.getInstance();
	}


	@Override
	public void init() {
		dt = new Drivetrain(hardwareMap);
		packet=new TelemetryPacket();
		packet.clearLines();
	}
	@Override
	public void loop() {
		sendTelemetry("Loop");
		dt.setTarget(new double[] {10,10,45});
		while(!notWithin(new double[] {10,10,45})){
			dt.updateMoveTo();
			sendTelemetry("Move to target");
		}
		requestOpModeStop();
		while(true){
			sendTelemetry("Done");
		}
	}
}
