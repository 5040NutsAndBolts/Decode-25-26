package org.firstinspires.ftc.teamcode.helpers.states;

import androidx.annotation.NonNull;

public abstract class Mechanism {
	public boolean waitWorthy = false;

	@NonNull
	@Override
	abstract public String toString();

	abstract public void update(@NonNull Object[] o);

	abstract boolean isFinished();
}
