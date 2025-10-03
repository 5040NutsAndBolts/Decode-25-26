package org.firstinspires.ftc.teamcode.helpers.easypathing;
import androidx.annotation.NonNull;
import java.util.ArrayList;

public class Path {
	public final ArrayList<Mechanism> mechanisms = new ArrayList<>();
	private final ArrayList<Object[]> states = new ArrayList<>();
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
	public Path(String name){
		this.name=name;
	}

	/**
	 * add states to the path
	 * format: [Mechanism, Object[] (positions, coordinates, used in update for mechanisms), Object[] (tolerances, extra conditions, used in isFinished for mechanisms)]
	 * @param states states to be added (Using a runnable object allows arbitrary logic to be run)
	 */
	public void queueStates(@NonNull ArrayList<Object[]> states) {
		for(Object[] state : states) {
			assert state.length == 3;
			assert Mechanism.class.isAssignableFrom((Class<?>) state[0]);
			assert state[1].getClass().isArray();
			assert state[2].getClass().isArray();
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
	 * add a mechanism to the path
	 * @param mechanism mechanisms to be added
	 */
	public void addMechanisms(@NonNull ArrayList<Mechanism> mechanism) {
		for(Mechanism m : mechanism)
			addMechanism(m);
	}

	/**
	 * update the path
	 */
	private short stateIter = 0;
	public void update() {
		if(stateIter >= states.size()) return;

		// Correctly get the target mechanism class from the nested array
		Class<?> targetMechanismClass = ((Class<?>[]) states.get(stateIter)[0])[0];

		for(Mechanism m : mechanisms) {
			// Use isInstance() for a safe and correct check
			if(targetMechanismClass.isInstance(m)) {
				// Allows arbitrary logic to be run
				if(states.get(stateIter)[1] instanceof Runnable)
					((Runnable) states.get(stateIter)[1]).run();
				else
					m.update((Object[]) states.get(stateIter)[1]);

				// We found and updated the correct mechanism for this state, so we can stop searching.
				break;
			}
		}
		// Increment state counter if the path is finished
		if(isFinished()) stateIter++;
	}

	/**
	 * checks if all waitWorthy mechanisms are finished
	 * @return true if all waitWorthy mechanisms are finished
	 */
	public boolean isFinished() {
		for(Mechanism m : mechanisms) {
			if(!m.isFinished((Object[]) states.get(stateIter)[2]) && m.waitWorthy)
				return false;
		}return true;
	}

	/**
	 * Exception thrown when a mechanism is not found in the path
	 */
	public static class MechanismNotFoundException extends Exception {
		/**
		 * creates a new MechanismNotFoundException
		 * @param c Class of mechanism that was not found
		 */
		public MechanismNotFoundException(@NonNull Class<?> c) {
			super("Mechanism of type " + c.getName() + " not found in mechanisms list");
		}
	}

	/**
	 * get a mechanism from the path's list of mechanisms
	 * @param c Class of mechanism to get
	 *
	 */
	@NonNull
	public Mechanism getMechanism(Class<?> c) throws MechanismNotFoundException {
		for(Mechanism m : mechanisms) {
			if(c.isAssignableFrom(m.getClass()))
				return m;
		}
		throw new MechanismNotFoundException(c);
	}

	/**
	 * get a mechanism from the path's list of mechanisms
	 * @param index index of mechanism in list to get
	 * @return Mechanism if found, null if not
	 */
	public Mechanism getMechanism(short index) {
		if(index >= mechanisms.size() || index < 0) throw new IndexOutOfBoundsException("Index " + index + " is out of bounds");
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