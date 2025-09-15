package org.firstinspires.ftc.teamcode.helpers.states;

import androidx.annotation.NonNull;

import java.util.ArrayList;

public class Path {
	public static final ArrayList<Mechanism> mechanisms = new ArrayList<>();
	private final ArrayList<Object[][]> states = new ArrayList<>();

	/**
	 * add states to the path
	 * @param states states to be added
	 */
	public void queueStates(@NonNull ArrayList<Object[][]> states) {
		for(Object[][] state : states) {
			assert state.length == 2;
			assert Mechanism.class.isAssignableFrom(state[0][0].getClass());
		}
		this.states.addAll(states);
	}

	/**
	 * add a mechanishm to the path
	 * @param mechanism mechanism to be added
	 */
	public void addMechanism(@NonNull Object mechanism) {
		//ensure that the mechanism is a subclass of Mechanism
		assert Mechanism.class.isAssignableFrom(mechanism.getClass());
		mechanisms.add((Mechanism) mechanism);
	}

	/**
	 * update the path
	 */
	private short stateIter = 0;
	public void update() {
		if(stateIter >= states.size()) return;
		for(Mechanism m : mechanisms) {
			//match state to mechanism
			if(m.getClass().equals(states.get(stateIter)[0][0]))
				m.update(states.get(stateIter)[1]);
		}
		//increment if done
		if(isFinished()) stateIter++;
	}

	public boolean isFinished() {
		for(Mechanism m : mechanisms) {
			if(!m.isFinished() && m.waitWorthy)
				return false;
		}return true;
	}

	@Override
	@NonNull
	public String toString() {
		StringBuilder s = new StringBuilder();
		mechanisms.forEach((m) -> s.append(m.toString()));
		return s.toString();
	}
}