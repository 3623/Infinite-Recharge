package frc.modeling;

import frc.controls.CubicSplineFollower.Waypoint;
import frc.util.Pose;

public class FieldPositions {

    // Left
    public static final Pose LEFT_START = new Pose(-3.3, -3.5, 0.0);
    public static final Waypoint LEFT1 = new Waypoint(-3.3, -5.8, 0.0, -0.8, true);
    public static final Waypoint LEFT2 = new Waypoint(0.5, -3.0, 50.0, 0.8, true);
    public static final Waypoint LEFT3 = new Waypoint(-0.2, -5.5, -25.0, -0.8, true);
    public static final Waypoint LEFT4 = new Waypoint(1.0, -3.0, 0.0, 0.8, true);

    // Middle
	public static final Pose MIDDLE_START = new Pose(2.5, -3.5, 0.0);
	public static final Waypoint MIDDLE1 = new Waypoint(1.7, -5.9, 65.0, -0.8, true);
	public static final Waypoint MIDDLE2 = new Waypoint(0.1, -5.9, 135.0, -0.8, false);
	public static final Waypoint MIDDLE3 = new Waypoint(1.0, -3.0, -140.0, -0.8, true);

    // Right
	public static final Pose RIGHT_START = new Pose(3.4, -3.5, 0.0);
	public static final Waypoint RIGHT1 = new Waypoint(3.4, -6.5, 0.0, -1.0, true);
	public static final Waypoint RIGHT2 = new Waypoint(3.4, -5.0, 0.0, 1.0, true);
	public static final Waypoint RIGHT3 = new Waypoint(3.4, -9.1, 0.0, -1.0, true);
	public static final Waypoint RIGHT4 = new Waypoint(2.7, -5.0, 20.0, 1.0, true);
	public static final Waypoint RIGHT5 = new Waypoint(1.6, -5.9, 70.0, -1.0, true);

    // Steal balls
	public static final Pose STEAL_START = new Pose(2.5, -3.0, 0.0);
	public static final Waypoint STEAL1 = new Waypoint(1.6, -5.9, 70.0, -1.0, true);
	public static final Waypoint STEAL2 = new Waypoint(-1.6, -8.8, -20.0, -1.0, false);
	public static final Waypoint STEAL3 = new Waypoint(-0.6, -9.6, -70.0, -1.0, false);
	public static final Waypoint STEAL4 = new Waypoint(1.0, -9.5, -110.0, -1.0, false);
	public static final Waypoint STEAL5 = new Waypoint(2.0, -4.5, -180.0, -1.0, true);

}
