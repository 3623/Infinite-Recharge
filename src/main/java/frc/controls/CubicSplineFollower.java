/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controls;

import java.util.ArrayList;

import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Tuple;
import frc.util.Utils;

/**
 * Add your docs here.
 */
public class CubicSplineFollower {
    private final double MAX_SPEED;
    private final double WHEEL_BASE;
    private final double UPDATE_RATE;

    private ArrayList<Waypoint> waypoints;
    private Waypoint curWaypoint;
    private int index = 0;

    public Boolean isFinished = false;

    private static final double kMaxAccelDefault = 0.1; // m/s^2 * 200
    private double kMaxAccel;
    private static final double kSlowdownRadiusCritical = 1.3;
    private static final double kMinApproachSpeedCritical = 0.2;
    private static final double kRadiusCriticalDefault = 0.05; // m
    private double kRadiusCritical; // m
    private static final double kScaleRadiusPathDefault = 0.1; // constant
    private double kScaleRadiusPath; // constant
    private double kRadiusPath = 0.0; // this updates dynamically
    private static final double kAngularErrorPathDefault = 5.0;
    // deg, keeping this because this dictates when the robot switches
    private double kAngularErrorPath;
    private double kMaxSplineAngle = Math.PI * 0.3;

    double ffSpeed = 0.0;

    private Boolean debug;

    public CubicSplineFollower(double robotMaxSpeed, double robotWheelBase, double updateRate, Boolean debug,
            double maxAccel, double goalRadius, double pathRadiusScale, double pathAngularError) {
        MAX_SPEED = robotMaxSpeed;
        WHEEL_BASE = robotWheelBase;

        UPDATE_RATE = updateRate;

        this.debug = debug;

        kMaxAccel = maxAccel;
        kRadiusCritical = goalRadius;
        kScaleRadiusPath = pathRadiusScale;
        kAngularErrorPath = pathAngularError;

        waypoints = new ArrayList<Waypoint>();
    }

    public CubicSplineFollower(double robotMaxSpeed, double robotWheelBase) {
        this(robotMaxSpeed, robotWheelBase, 200.0, false, kMaxAccelDefault, kRadiusCriticalDefault,
                kScaleRadiusPathDefault, kAngularErrorPathDefault);
    }

    /**
     * Updates the path follower with a new robot pose. Should be called at rate
     * equal to {@code UPDATE_RATE}.
     *
     * @param robotPose the current robot pose, with position and velocities
     * @return a tuple with left and right wheel voltages
     */
    public Tuple updatePursuit(Pose robotPose) {
        if (waypoints.size() < 1)
            return new Tuple(0.0, 0.0);
        curWaypoint = waypoints.get(index);
        double distanceFromWaypoint = Geometry.distance(robotPose, curWaypoint);
        ffSpeed = curWaypoint.kSpeed;
        if (curWaypoint.isCritical) { // important to be at exactly

            if (distanceFromWaypoint < Math.abs(ffSpeed) * kSlowdownRadiusCritical) {
                // speed reduces as distance gets smaller
                // TODO This is probably unnecesarry since FPID should be used now
                ffSpeed = Math.copySign(distanceFromWaypoint / kSlowdownRadiusCritical, ffSpeed);
                if (Math.abs(ffSpeed) < kMinApproachSpeedCritical) {
                    ffSpeed = Math.copySign(kMinApproachSpeedCritical, ffSpeed);
                }
            }
            if (distanceFromWaypoint < kRadiusCritical || isFinished) {
                ffSpeed = 0.0;
                // at point and heading, we're done
                if (!isFinished)
                    System.out.println("At Waypoint: " + index + " (" + curWaypoint.toString() + ")");
                if (index == waypoints.size() - 1 || isFinished) {
                    if (!isFinished)
                        System.out.println("Finished Path Following");
                    isFinished = true;
                    return new Tuple(0.0, 0.0);
                } else {
                    index++;
                    curWaypoint = waypoints.get(index);
                }

                // if it is at point but not heading, the code originally called PTR
                // Im deleting this a DrivetrainControls
                // What should be done is PTR becomes a function in Drivetrain
                // Then the command should dictate to use PTR or not
            }
        } else if (distanceFromWaypoint < kRadiusPath
                && Utils.withinThreshold(robotPose.heading, curWaypoint.heading, kAngularErrorPath)) {
            // at non-critical waypoint
            System.out.println("At Waypoint: " + index + " (" + curWaypoint.toString() + ")");
            index++;
            curWaypoint = waypoints.get(index);
        }
        // if not in a special case, just run path following
        return pathFollowing(robotPose);
    }

    /**
     * Uses a cubic spline calculated OTF to figure out a projected change in angle
     * required to follow path and uses this as a feed forward value in conjuction
     * with a d term used to cancel out rotational inertia of the robot. This method
     * cheats by setting the initial point of the cubic spline as x=0, y=0, dx=0 to
     * make calculations simpler. This means that the waypoint has to be converted
     * to local coordinates in reference to the robot.
     *
     * @return a tuple of left and right output linear speed
     */
    public Tuple pathFollowing(Pose robotPose) {
        Tuple pathCoefficients = getPathGeometry(robotPose, curWaypoint);
        double a = pathCoefficients.left;
        double b = pathCoefficients.right;
        double nextSpeed = ((MAX_SPEED * ffSpeed) * 0.1) + (robotPose.velocity * 0.9);
        double deltaX = nextSpeed / UPDATE_RATE;
        if (Math.signum(deltaX) != Math.signum(ffSpeed))
            deltaX = 0.0;
        /*
         * Average of ffSpeed and actual speed scaled by cosine (to account for how far
         * off straight the robot has to drive) and cos again (the further off straight
         * the longer the curve) then divided by update rate (to get deltaX, the
         * position along the spline the robot will be at for the next update, giving a
         * feed forward point). If this just used actual speed, a stopped robot would
         * not look ahead.
         */

        if (deltaX != 0.0) {
            double y2 = (a * deltaX * deltaX * deltaX) + (b * deltaX * deltaX);
            double hypot = Geometry.hypotenuse(deltaX, y2);
            double ratio = Math.abs(deltaX / hypot);
            deltaX *= ratio;
        }

        kRadiusPath = Math.abs(deltaX) * UPDATE_RATE * kScaleRadiusPath;
        double dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX);
        double relativeFFAngle = Math.atan(dx2);
        double omega = relativeFFAngle * UPDATE_RATE;

        // Convert from derivative to angle

        double desiredSpeed = ffSpeed * MAX_SPEED;
        if (desiredSpeed - robotPose.velocity > kMaxAccel)
            desiredSpeed = robotPose.velocity + kMaxAccel;
        else if (desiredSpeed - robotPose.velocity < -kMaxAccel)
            desiredSpeed = robotPose.velocity - kMaxAccel;
        double lrSpeedDifference = omega * WHEEL_BASE;
        double leftSpeed = desiredSpeed - (lrSpeedDifference / 2);
        double rightSpeed = desiredSpeed + (lrSpeedDifference / 2);
        return new Tuple(leftSpeed, rightSpeed);
    }

    /**
     * Calculates the relative angles and distances from the current robot position
     * to the desired goal point.
     *
     * @param startPoint the start position of the robot, if using dynamic path
     *                   generation, this should be the robot position
     * @param goalPoint  the goal position of the path to be calculated
     * @return a tuple of path coefficients a and b respectively for a cubic spline
     */
    private Tuple getPathGeometry(Pose startPoint, Pose goalPoint) {
        double distanceFromWaypoint = Geometry.distance(startPoint, goalPoint);
        double straightPathAngle = Math.atan2(goalPoint.x - startPoint.x, goalPoint.y - startPoint.y);
        double relativeAngle = startPoint.r - straightPathAngle;
        double relativeOpposDist = distanceFromWaypoint * Math.sin(relativeAngle);
        double relativeAdjacDist = distanceFromWaypoint * Math.cos(relativeAngle);
        double relativeGoalAngle = startPoint.r - goalPoint.r;
        relativeGoalAngle = Utils.limit(relativeGoalAngle, kMaxSplineAngle, -kMaxSplineAngle);
        double relativeGoalDeriv = Math.tan(relativeGoalAngle);
        if (debug) {
            System.out.println(relativeAdjacDist + " " + relativeOpposDist + " " + relativeGoalDeriv);
        }
        return generateSpline(relativeAdjacDist, relativeOpposDist, relativeGoalDeriv);
    }

    /**
     * Calculates the value of two coefficients (a & b) of a cubic spline specified
     * by two points and derivatives.
     *
     * @Note The first point is assumed to be (0, 0) with a derivative of 0. Second
     *       point must be in reference to this point
     * @param x  the x coordinate of the second point
     * @param y  the y coordinate of the second point
     * @param dx the desired slope of the second point
     * @implNote Not complicated, just two equations derived from solving the system
     *           of equations where x1=0, y1=0, and dx1=0, and x2, y2, and dx2 are
     *           specified in relation to p1, and y=ax^3+bx^2+cx+d (c and d are
     *           equal to 0 because of definition)
     * @return a tuple for coefficients a and b respectively
     */
    private static Tuple generateSpline(double x, double y, double dx) {
        double a = ((x * dx) - (2 * y)) / (x * x * x);
        double b = ((3 * y) - (dx * x)) / (x * x);
        return new Tuple(a, b);
    }

    /**
     * Clears the array list of waypoints and resets index so that the path follower
     * can be used again
     */
    public void clearWaypoints() {
        waypoints.clear();
        index = 0;
        isFinished = false;
    }

    /**
     * Returns the current waypoint being followed by the path follower
     *
     * @return {@link Waypoint}
     */
    public Waypoint getCurrentWaypoint() {
        return curWaypoint;
    }

    /**
     * Adds a waypoint to the list of waypoints (FILO)
     *
     * @param newWaypoint see {@link Waypoint}
     */
    public void addWaypoint(Waypoint newWaypoint) {
        waypoints.add(newWaypoint);
    }

    public void addWaypoint(double x, double y, double heading, double speed, Boolean isCritical) {
        waypoints.add(new Waypoint(x, y, heading, speed, isCritical));
    }

    /**
     * Contains information to define a point along a desired path
     */
    public static class Waypoint extends Pose {
        // public final Pose point;
        protected double kSpeed;
        protected Boolean isCritical;

        /**
         * Constructor for waypoint
         *
         * @param x        in meters
         * @param y        in meters
         * @param heading  in degrees. Call .r for radians
         * @param speed    in desired speed on a scale of -1 to 1
         * @param critical whether or not the waypoint is critical. Will stop at a
         *                 critical waypoint
         */
        public Waypoint(double x, double y, double heading, double speed, Boolean critical) {
            super(x, y, heading);
            this.kSpeed = speed;
            this.isCritical = critical;
        }

        public Waypoint(double x, double y, double heading) {
            this(x, y, heading, 0.0, false);
        }

        public Waypoint(Pose pose) {
            this(pose.x, pose.y, pose.heading, 0.0, false);

        }

        public Waypoint(double x, double y, double heading, double speed) {
            this(x, y, heading, speed, false);
        }

        @Override
        public String toString() {
            return "x: " + x + ", y: " + y + ", heading: " + heading + ", speed: " + kSpeed;
        }
    }

    public static void main(String[] args) {
    }
}
