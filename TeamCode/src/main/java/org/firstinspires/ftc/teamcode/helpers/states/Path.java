package org.firstinspires.ftc.teamcode.helpers.states;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class Path {
	public static final ArrayList<Mechanism> mechanisms = new ArrayList<>();
	private final ArrayList<Object[][]> states = new ArrayList<>();
	private final String name;

	/**
	 * create a new path
	 * @param mechanisms mechanisms to be used in the path
	 */
	public Path(@NonNull ArrayList<Mechanism> mechanisms, String name) {
		addMechanisms(mechanisms);
		this.name = name;
	}

	/**
	 * create a new path
	 * @param name name of the path for telemetry
	 */
	public Path(String name){this.name=name;}

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
	public static void addMechanism(@NonNull Object mechanism) {
		//ensure that the mechanism is a subclass of Mechanism
		assert Mechanism.class.isAssignableFrom(mechanism.getClass());
		mechanisms.add((Mechanism) mechanism);
	}

	/**
	 * add a mechanishm to the path
	 * @param mechanism mechanism to be added
	 */
	public static void addMechanisms(@NonNull ArrayList<Mechanism> mechanism) {
		for(Mechanism m : mechanism)
			addMechanism(m);
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

	/**
	 * get a mechanism from the path's list of mechanisms
	 * @param c Class of mechanism to get
	 * @return Mechanism if found, null if not
	 */
	public Mechanism getMechanism(Class<?> c) {
		for(Mechanism m : mechanisms) {
			if(c.isAssignableFrom(m.getClass()))
				return m;
		}
		return null;
	}

	/**
	 * get a mechanism from the path's list of mechanisms
	 * @param index index of mechanism in list to get
	 * @return Mechanism if found, null if not
	 */
	public Mechanism getMechanism(short index) {
		if(index >= mechanisms.size() || index < 0) return null;
		return mechanisms.get(index);
	}

	@Override
	@NonNull
	public String toString() {
		StringBuilder s = new StringBuilder();
		s.append(name).append("\n");
		mechanisms.forEach((m) -> s.append(m.toString()).append("\n"));
		return s.toString();
	}
}