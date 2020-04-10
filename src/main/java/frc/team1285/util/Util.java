package frc.team1285.util;

public class Util {
	public static double limitValue(double val) {
		return Util.limitValue(val, 1.0);
	}

	public static double limitValue(double val, double max) {
		return Util.limitValue(val, max, -max);
	}

	/**
	 * 
	 * @param val
	 * @param max
	 * @param min
	 * @return value between min and max
	 */
	public static double limitValue(double val, double max, double min) {
		if (val > max) {
			return max;
		} else if (val < min) {
			return min;
		} else {
			return val;
		}
	}

	/**
	 * 
	 * @param val
	 * @return squared of val while maintaining sign value
	 */
	public static double squareMaintainSign(double val) {
		return Math.signum(val) * val * val;
	}

	public static double calcLeftTankDrive(double x, double y) {
		return (y + x);
	}

	public static double calcRightTankDrive(double x, double y) {
		return (y - x);
	}

	public static double absMax(double a, double b, double c) {
		a = Math.abs(a);
		b = Math.abs(b);
		c = Math.abs(c);
		if (a > b && a > c) {
			return a;
		} else if (b > c) {
			return b;
		} else {
			return c;
		}
	}

	public static boolean allWithinRange(double[] values, double target, double eps) {
		boolean withinRange = true;

		for (int i = 0; i < values.length; i++) {
			withinRange &= isWithinRange(target, values[i], eps);
		}

		return withinRange;

	}

	public static boolean isWithinRange(double target, double val, double eps) {
		if (Math.abs(val - target) <= eps) {
			return true;
		} else {
			return false;
		}
	}
}
