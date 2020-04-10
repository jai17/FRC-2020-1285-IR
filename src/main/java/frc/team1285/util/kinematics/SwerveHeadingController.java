package frc.team1285.util.kinematics;

import frc.team1285.util.PIDControllerF;
import edu.wpi.first.wpilibj.Timer;

public class SwerveHeadingController {
	private double targetHeading;
	private double disabledTimestamp;
	private final double disableTimeLength = 0.2;
	private PIDControllerF stabilizationPID;
	private PIDControllerF snapPID;
	private PIDControllerF stationaryPID;

	public enum State {
		Off, Stabilize, Snap, TemporaryDisable, Stationary
	}

	private State currentState = State.Off;

	public State getState() {
		return currentState;
	}

	private void setState(State newState) {
		currentState = newState;
	}

	public SwerveHeadingController() {
		stabilizationPID = new PIDControllerF(0.005, 0.0, 0.0005, 0.0, 1);
		snapPID = new PIDControllerF(0.015, 0.0, 0.0, 0.0, 1);
		stationaryPID = new PIDControllerF(0.01, 0.0, 0.002, 0.0, 1);

		targetHeading = 0;
	}

	public void setStabilizationTarget(double angle) {
		targetHeading = angle;
		setState(State.Stabilize);
	}

	public void setSnapTarget(double angle) {
		targetHeading = angle;
		setState(State.Snap);
	}

	public void setStationaryTarget(double angle) {
		targetHeading = angle;
		setState(State.Stationary);
	}

	public void disable() {
		setState(State.Off);
	}

	public void temporarilyDisable() {
		setState(State.TemporaryDisable);
		disabledTimestamp = Timer.getFPGATimestamp();
	}

	public double getTargetHeading() {
		return targetHeading;
	}

	public double updateRotationCorrection(double heading, double timestamp) {
		double correction = 0;
		double error = heading - targetHeading;

		switch (currentState) {
			case Off:

				break;
			case TemporaryDisable:
				targetHeading = heading;
				if (timestamp - disabledTimestamp >= disableTimeLength)
					setState(State.Stabilize);
				break;
			case Stabilize:
				correction = stabilizationPID.calcPIDError(error);
				break;
			case Snap:
				correction = snapPID.calcPIDError(error);
				break;
			case Stationary:
				correction = stationaryPID.calcPIDError(error);
				break;
		}

		return correction;
	}

}
