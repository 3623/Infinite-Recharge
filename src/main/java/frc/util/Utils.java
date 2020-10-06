package frc.util;

public class Utils {

	public static Boolean withinThreshold(double value, double goal, double epsilon) {
		return Math.abs(value - goal) <= epsilon;
	}

	public static Boolean outsideDeadband(double value, double center, double deadband) {
		return Math.abs(value - center) >= deadband;
	}

	public static double limit(double value, double upperBound, double lowerBound) {
		return Math.max(lowerBound, Math.min(upperBound, value));
	}

	public static double limit(double value) {
		return limit(value, 1.0, -1.0);
	}

	public static double applyDeadband(double value, double deadband) {
		if (Utils.withinThreshold(value, 0.0, deadband)) {
			return 0.0;
		} else {
			return value;
		}
	}

	/**
	 * Takes an angle and converts it to base degrees (-180 - 180)
	 * @param angle angle in degrees
	 * @return limited angle in degrees
	 */
	public static double limitAngleDegrees(double angle){
		return limitAngle(angle, 180.0);
	}

	/**
	 * Takes an angle and converts it to base degrees (-180 - 180)
	 *
	 * @param angle angle in degrees
	 * @return limited angle in degrees
	 */
	public static double limitAngleRadians(double angle) {
		return limitAngle(angle, Math.PI);
	}

	private static double limitAngle(double angle, double maxAngle) {
		return (((angle + maxAngle) % (2*maxAngle)) + (2*maxAngle)) % (2*maxAngle) - maxAngle;
	}
}