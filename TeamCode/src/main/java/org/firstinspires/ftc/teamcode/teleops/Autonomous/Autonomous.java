package org.firstinspires.ftc.teamcode.teleops.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.helpers.states.Mechanism;
import org.firstinspires.ftc.teamcode.helpers.states.Path;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

import java.util.ArrayList;

public class Autonomous extends OpMode {
	public ArrayList<Path> paths;
	public ArrayList<Object> mechanisms;


	public void update() {
		for(Path p : paths)
			while(!p.isFinished())
				p.update();
	}

	@Override
	public void init() {
		mechanisms.add(new Drivetrain(hardwareMap));


		Path p = new Path();
		mechanisms.forEach((m) -> p.mechanisms.add((Mechanism) m));

	}

	@Override
	public void loop() {

	}
}
