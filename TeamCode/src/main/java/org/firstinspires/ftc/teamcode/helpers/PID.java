package org.firstinspires.ftc.teamcode.helpers;
import androidx.annotation.NonNull;
import java.util.function.Supplier;

public class PID {
	private final double kp, ki, kd,  teleopGain;
	private double currentTarget, errorSum, lastOutput, lastError;
	private long lastTime;
	private int deltaTime;
	private double currentOutput;
	private final Supplier<Double> getCurrent;

	/**
	 * Creates a PID controller
	 * @param kp Proportional gain (Gets near position faster, more prone to overshoot)
	 * @param ki Integral gain (Decreases steady-state error, more oscillative, less responsive)
	 * @param kd Derivative gain (Reduces overshoot and smooths response)
	 * @param getCurrent Function that returns the current value of the variable being controlled
	 * @param teleopGain adjust rate of change of target based on stickPower in teleopControl()
	 */
	public PID(double kp, double ki, double kd, Supplier<Double> getCurrent, double teleopGain) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.getCurrent = getCurrent;
		this.teleopGain = teleopGain;
		lastTime = System.currentTimeMillis();
	}

	/**
	 * Barebones,mostly for autos
	 * @param kp Proportional gain (Gets near position faster, more prone to overshoot)
	 * @param ki Integral gain (Decreases steady-state error, more oscillative, less responsive)
	 * @param kd Derivative gain (Reduces overshoot and smooths response)
	 */
	public PID (double kp, double ki, double kd) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		lastTime = System.currentTimeMillis();
		getCurrent = null;
		teleopGain = 0;
	}

	//Calculates power output
	private double calculate() {
		double currentPosition = getCurrent.get();

		//If deltaTime is too small, return last output to minimize floating point errors
		if(deltaTime < 5)
			return lastOutput;


		double currentError = currentTarget - currentPosition;
		errorSum += currentError;

		//Instantaneous error
		double Proportional = kp * currentError;
		//Error over time
		double Integral = ki * errorSum  *deltaTime;
		//Rate of change of error
		double Derivative = kd * (currentError - lastError) / deltaTime;

		double output = Proportional + Integral + Derivative;

		//Update persistent values
		lastError = currentError;
		lastOutput = output;
		lastTime = System.currentTimeMillis();

		currentOutput = output;
		return output;
	}
	private double calculate(double currentPosition) {
		//If deltaTime is too small, return last output to minimize floating point errors
		if(deltaTime < 3)
			return lastOutput;


		double currentError = currentTarget - currentPosition;
		errorSum += currentError;

		//Instantaneous error
		double Proportional = kp * currentError;
		//Error over time
		double Integral = ki * errorSum * deltaTime;
		//Rate of change of error
		double Derivative = kd * (currentError - lastError) / deltaTime;

		double output = Proportional + Integral + Derivative;

		//Update persistent values
		lastError = currentError;
		lastOutput = output;
		lastTime = System.currentTimeMillis();

		currentOutput = output;
		return output;
	}

	public double teleOpControl(double stickPower) {
		updateDeltaTime();
		/*Adds stick power adjusted via teleopGain to increase
		  or decrease target, multiply by deltaTime to normalize
		  over time differences between clock cycles*/
		setTarget(Math.abs(stickPower) > .05 ? (currentTarget + stickPower * deltaTime * teleopGain) : currentTarget);
		return calculate();
	}


	public double autoControl () {
		updateDeltaTime();
		return calculate();
	}
	public double autoControl (double current) {
		updateDeltaTime();
		return calculate(current);
	}

	/**
	 * Sets the target value for the PID controller
	 * @param target desired target value
	 */
	public void setTarget(double target) {
		currentTarget = target;
	}

	//Subtract current time from last time to get delta time
	private void updateDeltaTime() {
		long cur = System.currentTimeMillis();
		deltaTime = (int) (cur - lastTime);
		lastTime = cur;
	}

	//Return current target
	public double getTarget() {
		return currentTarget;
	}

	@NonNull
	@Override
	public String toString() {
		return
			"PID Controller: \n" +
			"\tKp: " + kp + "\n" +
			"\tKi: " + ki + "\n" +
			"\tKd: " + kd+ "\n" +
			"\tTeleop Gain: " + teleopGain + "\n" +
			"\tcurrentOutput: " + currentOutput+ "\n" +
			"\tcurrentTarget: " +currentTarget;
	}
}