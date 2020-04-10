package frc.team1285.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDControllerF extends PIDController {

	private double eps;

	private double feedForward;

	/**
	 * PIDControllerF
	 * 
	 * Constructor for the PID Controller
	 * 
	 * @param p   double
	 * @param i   double
	 * @param d   double
	 * @param f   double
	 * @param eps double
	 */
	public PIDControllerF(double p, double i, double d, double f, double eps) {
		super(p, i, d, eps);
		this.eps = eps;
		this.feedForward = f;
	}

	@Override
	public double calcPID(double current) {
		double feedForwardOutput = (super.getDesiredVal() * this.feedForward);
		if (this.debug) {
			SmartDashboard.putNumber("FF out", feedForwardOutput);
		}
		return super.calcPID(current) + feedForwardOutput;
	}

	public void setConstants(double p, double i, double d, double f) {
		super.setConstants(p, i, d, eps);
		this.feedForward = f;
	}

	@Override
	public boolean isDone() {
		double currError = Math.abs(this.previousError);

		// close enough to target
		if (currError <= this.finishedRange) {
			return true;
		} else {
			return false;
		}
	}

	public void setFeedForward(double f) {
		this.feedForward = f;
	}

}
