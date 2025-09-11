package org.firstinspires.ftc.teamcode.helpers.states;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

import java.util.ArrayList;

public class Path {
	public final ArrayList<Mechanism> mechanisms = new ArrayList<>();

	private ArrayList<Object[][]> states;

	/**
	 * add a state to the path
	 * @param mechanismClass class of desired mechanism to add a state to
	 * @param state desired state of the mechanism
	 */
	public void queueState(Class<?> mechanismClass, Object[] state) {
		states.add(new Object[][]{{mechanismClass}, state});
	}


	public void update() {
		for(Object[][] s : states)
			for(Mechanism m : mechanisms) {
				if (Mechanism.class.isAssignableFrom((Class<?>) s[0][0])) {
					if(m.isFinished() && !m.waitWorthy)
						continue;
					m.update(s[1]);
				}
			}
	}

	public boolean isFinished() {return false;}
}