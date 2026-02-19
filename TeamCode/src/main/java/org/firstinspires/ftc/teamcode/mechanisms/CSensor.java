package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Array;
import java.util.Arrays;

public class CSensor {
	private ColorSensor cs;
	public CSensor(String name, HardwareMap hardwareMap) {
		cs = hardwareMap.get(ColorSensor.class, name);
	}

	public Lights.Color getColor() {
		int[] rgb = {cs.red(), cs.green(), cs.blue()};

		int oScore = (int) (255-((rgb[0] + rgb[1] + rgb[2]) / 255.0));
		int gScore = (int) (rgb[1] - rgb[0]*.15-rgb[2]*.1);
		int pScore = (int)((rgb[2] * - rgb[1]*.15));

		if(gScore > oScore && gScore > pScore) {
			return Lights.Color.GREEN;
		} else if (pScore > oScore && pScore > gScore) {
			return Lights.Color.PURPLE;
		}else return Lights.Color.ORANGE;
	}

	@NonNull
	@Override
	public String toString() {
		int[] rgb = {cs.red(), cs.green(), cs.blue()};

		int oScore = (int) (255-((rgb[0] + rgb[1] + rgb[2]) / 255.0));
		int gScore = (int) ((rgb[1] - rgb[0]*.15-rgb[2]*.1)/255.0);
		int pScore = (int)((rgb[2] - rgb[1]*.15) / 255.0);

		int[] scored = {oScore, gScore, pScore};
		return Arrays.toString(rgb)+ "  |  "+Arrays.toString(scored)+ "\n";
	}
}