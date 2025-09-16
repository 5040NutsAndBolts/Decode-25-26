package org.firstinspires.ftc.teamcode.helpers.states;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Mechanism {
	//Might be used if we want like a lift to hit the bottom but it doesn't need to be done by the end of the path
	public boolean waitWorthy = false;

	@NonNull
	@Override
	abstract public String toString();

	abstract public void update(@NonNull Object[] o);

	protected abstract boolean isFinished();
}