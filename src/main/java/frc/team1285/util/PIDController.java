package frc.team1285.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDController {
	private double pConst;
	private double iConst;
	private double dConst;
	private double desiredVal;
	protected double previousError;
	private double errorSum;
	protected double finishedRange;
	private double maxOutput;
	private double minOuput;
	private int minCycleCount;
	private int currentCycleCount;
	private boolean firstCycle;
	protected boolean debug;
	private double lastTime;
	private double deltaTime;
	private double iRange;

	/**
	 * PIDController
	 * 
	 * Constructor for the PID Controller
	 * 
	 * @param p        double
	 * @param i        double
	 * @param d        double
	 * @param epsRange double
	 */
	public PIDController(double p, double i, double d, double epsRange) {
		this.pConst = p;
		this.iConst = i;
		this.dConst = d;
		this.finishedRange = epsRange;
		this.desiredVal = 0.0;
		this.firstCycle = true;
		this.maxOutput = 1.0;
		this.currentCycleCount = 0;
		this.minCycleCount = 5;
		this.debug = false;
		this.minOuput = 0;
		this.iRange = 1000; // Default high number to always apply I
	}

	/**
	 * setConstants
	 * 
	 * reset the constants for the PID Controller
	 * 
	 * @param p   double
	 * @param i   double
	 * @param d   double
	 * @param eps double
	 */
	public void setConstants(double p, double i, double d, double eps) {
		this.pConst = p;
		this.iConst = i;
		this.dConst = d;
		this.setFinishedRange(eps);
	}

	/**
	 * setDesiredValue
	 * 
	 * sets the setpoint value for the PID Controller
	 * 
	 * @param val double
	 */
	public void setDesiredValue(double val) {
		this.desiredVal = val;
	}

	/**
	 * setFinishedRange
	 * 
	 * sets the finished range
	 * 
	 * @param range double
	 */
	public void setFinishedRange(double range) {
		this.finishedRange = range;
	}

	public double getFinishedRange() {
		return this.finishedRange;
	}

	public void enableDebug() {
		this.debug = true;
	}

	public void disableDebug() {
		this.debug = false;
	}

	public void setMaxOutput(double max) {
		this.maxOutput = max;
	}

	public void setMinMaxOutput(double min, double max) {
		this.maxOutput = max;
		this.minOuput = min;
	}

	public void setMinDoneCycles(int num) {
		this.minCycleCount = num;
	}

	public void resetErrorSum() {
		this.errorSum = 0.0;
	}

	public double getDesiredVal() {
		return this.desiredVal;
	}

	public void setIRange(double iRange) {
		this.iRange = iRange;
	}

	public double getIRange() {
		return this.iRange;
	}

	/**
	 * calcPID
	 * 
	 * calculate motor output given current sensor values
	 * 
	 * @param current double
	 * @return output double
	 */
	public double calcPID(double current) {
		return calcPIDError(this.desiredVal - current);
	}

	/**
	 * calcPIDError
	 * 
	 * calculate the motor outpur given the target error
	 * 
	 * @param error
	 * @return output double
	 */
	public double calcPIDError(double error) {
		double pVal = 0.0;
		double iVal = 0.0;
		double dVal = 0.0;

		if (this.firstCycle) {
			this.previousError = error;
			this.firstCycle = false;
			this.lastTime = System.currentTimeMillis();
			this.deltaTime = 20.0;
		} else {
			double currentTime = System.currentTimeMillis();
			this.deltaTime = currentTime - lastTime;
			this.lastTime = currentTime;
		}

		this.deltaTime = (this.deltaTime / 20.0); // 20ms is normal and should be 1

		/////// P Calc///////
		pVal = this.pConst * error;

		/////// I Calc///////
		if (Math.abs(error) < Math.abs(this.iRange)) { // Within desired range for using I
			this.errorSum += error * this.deltaTime;
		} else {
			this.errorSum = 0.0;
		}
		iVal = this.iConst * this.errorSum;

		/////// D Calc///////
		double deriv = (error - this.previousError) / this.deltaTime;
		dVal = this.dConst * deriv;

		// overal PID calc
		double output = pVal + iVal + dVal;

		// limit the output
		output = Util.limitValue(output, this.maxOutput);

		if (output > 0) {
			if (output < this.minOuput) {
				output = this.minOuput;
			}
		} else {
			if (output > -this.minOuput) {
				output = -this.minOuput;
			}
		}

		// store current value as previous for next cycle
		this.previousError = error;

		if (this.debug) {
			SmartDashboard.putNumber("P out", pVal);
			SmartDashboard.putNumber("I out", iVal);
			SmartDashboard.putNumber("D out", dVal);
			SmartDashboard.putNumber("PID OutPut", output);
			SmartDashboard.putNumber("derivative", deriv);
			SmartDashboard.putNumber("error", error);
			SmartDashboard.putNumber("prev error", previousError);
		}
		return output;
	}

	/**
	 * isDone
	 * 
	 * returns wether the PIDController is done
	 * 
	 * @return boolean
	 */
	public boolean isDone() {
		double currError = Math.abs(this.previousError);

		// close enough to target
		if (currError <= this.finishedRange) {
			this.currentCycleCount++;
		}
		// not close enough to target
		else {
			this.currentCycleCount = 0;
		}

		return this.currentCycleCount > this.minCycleCount;
	}

	public boolean getFirstCycle() {
		return this.firstCycle;
	}

	/**
	 * reset
	 * 
	 * reset the PID Controller before the next setpoint
	 */
	public void reset() {
		this.currentCycleCount = 0;
		this.firstCycle = true;
	}

	public void resetPreviousVal() {
		this.firstCycle = true;
	}

}
