package frc.robot.subsystems;

import frc.modeling.motors.CIMMotor;
import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Tuple;
import frc.util.Utils;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Model for a differential drivetrain, simulates mass and motor performance
 * Integrates acceleration over time for both sides of the drivetrain and uses
 * velocity to calculate the instantaneous curvature of the robot, which is then
 * integrated for position
 *
 * @author eric
 *
 */
public class DrivetrainModel {

	private static final double DRIVETRAIN_MASS = 63.5; // kg
	public static final double MAX_SPEED = 3.3;
	public static final double WHEEL_BASE = 0.67; // meters
	private static final double CENTER_MASS = 0.335; // from left wheel
	private Boolean COAST_MODE = false;
	public Pose center;

	private DrivetrainSide left, right;
	static final double WHEEL_RADIUS = 0.0774; // meters
	private static final double CIMS_PER_SIDE = 2.0; // Minicim is 0.58
	private static final double GEAR_RATIO = 10.75 / 1.0; // Reduction
	private static final double DRIVETRAIN_FRICTION = 115;
	private static final double MAX_FORCE = 200.0; // 250 is still under max, but 200 is conservative
	private static final double MAX_TORQUE = MAX_FORCE * WHEEL_RADIUS;

	public DrivetrainModel() {
		center = new Pose(0.0, 0.0, 0.0); // Initial robot position
		left = new DrivetrainSide();
		right = new DrivetrainSide();

		if (COAST_MODE) {
			left.coast = true;
			right.coast = true;
		} else {
			left.coast = false;
			right.coast = false;
		}
	}

	/**
	 * Sets the drivetrain position to a known location
	 *
	 * @param x        location (side to side), in meters
	 * @param y        location (forwards backwards, in meters
	 * @param heading, in degrees
	 */
	public void setPosition(double x, double y, double heading) {
		center.x = x;
		center.y = y;
		center.setHeading(heading);
		zero();
	}

	/**
	 * Zeros the velocity and acceleration of each drivetrain side
	 */
	public void zero() {
		left.velocity = 0.0;
		right.velocity = 0.0;
		left.acceleration = 0.0;
		right.acceleration = 0.0;
	}

	/**
	 * Sets the speed of each drivetrain side to a known value and calculates
	 * acceleration. For using encoders
	 *
	 * @param lSpeed left speed, in meters/sec
	 * @param rSpeed right speed, in meters/sec
	 * @param time   elapsed time since last update, in seconds
	 */
	public void updateSpeed(double lSpeed, double rSpeed, double time) {
		left.updateSpeed(lSpeed, time);
		right.updateSpeed(rSpeed, time);
	}

	/**
	 * Updates the models heading with a set heading. For using a gyro to more
	 * accurately track heading
	 *
	 * @param angle in degrees
	 */
	public void updateHeading(double heading) {
		center.setHeading(heading);
	}

	/**
	 * Updates velocity and acceleration of each side of the drivetrain using motor
	 * curves
	 *
	 * @param left  voltage
	 * @param right voltage
	 * @param the   elapsedtime between updates, in seconds
	 */
	public void updateVoltage(double lVoltage, double rVoltage, double time) {
		left.updateVoltage(lVoltage, time);
		right.updateVoltage(rVoltage, time);
	}

	/**
	 * Updates the models position from each sides velocity
	 *
	 * @param the elapsed time between updates, in seconds
	 */
	public void updatePosition(double time) {
		double radius = radiusICC(WHEEL_BASE, left.velocity, right.velocity);
		double omega = velocityICC(WHEEL_BASE, left.velocity, right.velocity);
		double theta = omega * (time);
		double sinTheta = Math.sin(theta);
		double alpha = ((Math.PI) - theta) / 2.0;
		double sinAlpha = Math.sin(alpha);

		double movementAngle = center.r + theta;
		double movement = Geometry.sideFromLawOfSines(radius, sinAlpha, sinTheta);

		if (omega == 0.0) {
			movement = -(left.velocity + right.velocity) / 2.0 * time;
		}
		double sine = Math.sin(movementAngle);
		double cosine = Math.cos(movementAngle);
		double movementX = -movement * sine;
		double movementY = -movement * cosine;
		center.update(movementX, movementY, -theta);

		center.velocity = (left.velocity + right.velocity) / 2.0;
		center.angularVelocity = -Math.toDegrees(theta) / time;

		// // Debug statements
		// System.out.println("X: " + center.x + ", Y: " + center.y + ", Heading: " +
		// center.heading);
		// System.out.println("MA: " + movementAngle + ", Rad ICC: " + radius);
		// System.out.println("Movement X: " + movementX + ", Movement Y: " +
		// movementY);
	}

	/**
	 * Limits acceleration using the models velocity and information about motors
	 *
	 * @param unchecked output voltage
	 * @return checked voltage, limited to acceleration of MAX_TORQUE constant
	 */
	public Tuple limitAcceleration(Tuple out) {
		double leftVoltage = out.left;
		double rightVoltage = out.right;
		double voltageDif = (leftVoltage - rightVoltage) / 2.0;
		double avg = (left.limitAcceleration(leftVoltage) + right.limitAcceleration(rightVoltage)) / 2.0;
		double leftVoltageChecked = left.limitAcceleration(leftVoltage) + voltageDif;
		double rightVoltageChecked = right.limitAcceleration(rightVoltage) - voltageDif;
		leftVoltageChecked = avg + voltageDif;
		// leftVoltageChecked = left.limitAcceleration(leftVoltageChecked);
		rightVoltageChecked = avg - voltageDif;
		// rightVoltageChecked = right.limitAcceleration(rightVoltageChecked);
		// rightVoltageChecked = right.limitAcceleration(rightVoltageChecked);

		// Boolean isLeftGreater = Math.abs(voltageDif) == voltageDif;
		// Boolean isLeftNeg = Math.abs(leftVoltageChecked) > leftVoltageChecked;
		// Boolean isRightNeg = Math.abs(rightVoltageChecked) > rightVoltageChecked;

		Tuple checkedOutput = new Tuple(leftVoltageChecked, rightVoltageChecked);
		return checkedOutput;
	}

	/**
	 * Limits acceleration using the models velocity and information about motors
	 *
	 * @param unchecked output voltage
	 * @return checked voltage, limited to acceleration of MAX_TORQUE constant
	 */
	public Tuple limitAcceleration(double leftOut, double rightOut) {
		return limitAcceleration(new Tuple(leftOut, rightOut));
	}

	private static class DrivetrainSide {
		double velocity;
		double acceleration;
		private double psuedoMass;
		private Boolean coast;
		// private CIMMotor cim = new CIMMotor();

		public DrivetrainSide() {
			velocity = 0.0;
			acceleration = 0.0;
			psuedoMass = DRIVETRAIN_MASS / 2;
			coast = false;
		}

		/**
		 * Sets the speed of each drivetrain side to a known value and calculates
		 * acceleration. For using encoders
		 *
		 * @param speed, in meters/sec
		 */
		protected void updateSpeed(double speed, double time) {
			double deltaVelocity = this.velocity - speed;
			this.acceleration = deltaVelocity / time;
			this.velocity = speed;
		}

		/**
		 * Updates velocity and acceleration of each side of the drivetrain using motor
		 * curves
		 *
		 * @param voltage
		 * @param the     elapsedtime between updates, in seconds
		 */
		protected void updateVoltage(double voltage, double time) {
			double motorSpeed = this.wheelSpeedToMotorSpeed(this.velocity);
			// double newAcceleration = this.wheelAcceleration(voltage, motorSpeed);

			double totalTorque = CIMMotor.outputTorque(voltage, motorSpeed) * GEAR_RATIO * CIMS_PER_SIDE;

			if (coast && Utils.withinThreshold(voltage, 0.0, 0.05))
				totalTorque = 0.0;
			double wheelForce = (totalTorque / WHEEL_RADIUS);

			double wheelnetForce = frictionModel(wheelForce, this.velocity);

			double newAcceleration = wheelnetForce / psuedoMass;
			this.velocity += (newAcceleration + this.acceleration) / 2 * time; // Trapezoidal integration
			this.acceleration = newAcceleration;
		}

		/**
		 * Limits acceleration using the models velocity and information about motors
		 *
		 * @param unchecked output voltage
		 * @return checked voltage, limited to acceleration of MAX_TORQUE constant
		 */
		protected double limitAcceleration(double outputVoltage) {
			double motorSpeed = this.wheelSpeedToMotorSpeed(this.velocity);
			double maxVoltage = CIMMotor.torqueToVoltage(MAX_TORQUE / CIMS_PER_SIDE / GEAR_RATIO, motorSpeed);
			double minVoltage = CIMMotor.torqueToVoltage(-MAX_TORQUE / CIMS_PER_SIDE / GEAR_RATIO, motorSpeed);

			double limitedVoltage = Utils.limit(outputVoltage, maxVoltage, minVoltage);

			// System.out.println("Max: " + maxVoltage + ", Min: " + minVoltage + ",
			// Limited: " + limitedVoltage);

			return limitedVoltage;
		}

		/**
		 * Converts linear wheel speed back to motor angular speed
		 *
		 * @param speed meters/sec
		 * @return angular speed, revolutions per minute
		 */
		private double wheelSpeedToMotorSpeed(double speed) {
			double wheelCircum = WHEEL_RADIUS * 2 * Math.PI;
			double wheelRevs = speed / wheelCircum * 60.0;
			double motorRevs = wheelRevs * GEAR_RATIO;
			return motorRevs;
		}

		/**
		 * Models friction as a constant
		 *
		 * @param Ideal output force of the drivetrain, in newtons
		 * @param Speed of the drivetrain, to set direction of friction
		 */
		private double frictionModel(double force, double speed) {
			double netForce;
			if (speed == 0.0) {
				netForce = force;
			} else if (speed < 0.0) {
				netForce = force + DRIVETRAIN_FRICTION;
			} else if (speed > 0.0) {
				netForce = force - DRIVETRAIN_FRICTION;
			} else {
				netForce = 0.0;
			}
			return netForce;
		}
	}

	/**
	 * The radius of the robot about the Instantaneous Center of Curvature (ICC)
	 * Used to infinitesimally calculate the displacement of the robot A positive
	 * radius is to the right of the robot (relative to robot) and negative is left
	 *
	 * @see <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">
	 *      Columbia University: CS W4733 NOTES - Differential Drive Robots</a>
	 * @param wheelBase width between left and right sides of the drivetrain, meters
	 * @param left      velocity of the left wheel, m/s
	 * @param right     velocity of the right wheel, m/s
	 * @return the radius from the center of the robot to the ICC
	 */
	private static double radiusICC(double wheelBase, double left, double right) {
		return -(wheelBase / 2) * (left + right) / (right - left);
	}

	/**
	 * The angular velocity of the robot about the Instantaneous Center of Curvature
	 * (ICC) Used to infinitesimally calculate the displacement of the robot A
	 * positive radius is to the left of the robot (relative to robot) and negative
	 * is right
	 *
	 * @see <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">
	 *      Columbia University: CS W4733 NOTES - Differential Drive Robots</a>
	 * @param wheelBase width between left and right sides of the drivetrain, meters
	 * @param left      velocity of the left wheel, m/s
	 * @param right     velocity of the right wheel, m/s
	 * @return
	 */
	private static double velocityICC(double wheelBase, double left, double right) {
		return (right - left) / wheelBase;
	}

	// For Testing
	public static void main(String[] args) {
		DrivetrainModel model = new DrivetrainModel();
		for (int i = 0; i < 1; i++) {
			model.updateSpeed(3.0, 3.0, 0.02);
			model.limitAcceleration(12.0, 12.0);
		}
	}
}
