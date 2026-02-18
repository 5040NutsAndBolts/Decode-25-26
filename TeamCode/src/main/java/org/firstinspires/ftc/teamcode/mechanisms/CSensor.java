package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CSensor {
	private ColorSensor cs;
	public CSensor(String name, HardwareMap hardwareMap) {
		cs = hardwareMap.get(ColorSensor.class, name);
	}

	public Lights.Color getColor() {
		int[] rgb = {cs.red(), cs.green(), cs.blue()};

		int oScore = (int)(((rgb[0] > 100 ? rgb[0] : 0) + rgb[1] * 1.2) * 100);
		int gScore = (int)((rgb[1] * 1.4 - rgb[0]*.15 - rgb[2]*.1) * 100);
		int pScore = (int)((rgb[2] * 1.2 - rgb[1]*.15) * 100);

		if(gScore > oScore && gScore > pScore) {
			return Lights.Color.GREEN;
		} else if (pScore > oScore && pScore > gScore) {
			return Lights.Color.PURPLE;
		}else return Lights.Color.ORANGE;
	}

}
