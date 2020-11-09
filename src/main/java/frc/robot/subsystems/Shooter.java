/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.modeling.FieldPositions;
import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Utils;

public class Shooter extends TerribleSubsystem {
    protected final int UPDATE_RATE = 200;

    private Turret turret;
    private Feeder feeder;
    private Hood hood;
    private Flywheel flywheel;

    private static final double AIM_THRESHOLD = 2.0;
    // degrees, +/- that shooter will stil aim for inner port, outside it will shoot at target
    private static final double INNER_GOAL_AIM_THRESHOLD = 20.0;

    private Pose robotLoc;
    private Pose targetPose = FieldPositions.OUR_GOAL;

    private double targetDistance = 0.0;
    private double targetAngle = 0.0; // global coordinates
    private double innerPortOffset = 0.0;
    private boolean shooting = false;
    private boolean readyToFire = false;
    private boolean fullyAimed = false;

    NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry targetX = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)
    NetworkTableEntry targetY = Lime.getEntry("ty"); // Vertical Offset From Crosshair to Target (-20.5 to 20.5 degrees)
    NetworkTableEntry ta = Lime.getEntry("ta"); // Target Area (0% of Image to 100% of Image)
    NetworkTableEntry targets = Lime.getEntry("tv"); // Valid Targets (0 or 1)

    private final double LIMELIGHT_ELEVATION_OFFSET = 20.0;
    private final double TARGET_RELATIVE_HEIGHT = 2.0; // meters
    private final double TARGET_WIDTH = 1.0; // m
    private final double LIMELIGHT_FOV = 54.0;

	private enum ShooterControlState {
        BLIND_AIM, // odometry based turn turret to target
        VISION_TRACKING, // when target has been found
        DISABLED,
    }
    private ShooterControlState controlState = ShooterControlState.DISABLED;

    private enum Target {
        GOAL,
        // LOADING_ZONE, // Maybe these will be used for localization..
        ENEMY_GOAL,
        // ENEMY_LOADING
    }
    private Target target = Target.GOAL;

    public Shooter() {
        turret = new Turret();
        hood = new Hood();
        feeder = new Feeder();
        flywheel = new Flywheel();

        this.updateThreadStart();
    }

    /**
     * Begins blind aim and vision tracking for target.
     * It the target is our goal, then will prepare to fire
     * @param target1
     */
    public void seekTarget(Target target1) {
        target = target1;
        if (target == Target.GOAL) {
            shooting = true;
            targetPose = FieldPositions.OUR_GOAL;
        } else if (target == Target.ENEMY_GOAL) {
            targetPose = FieldPositions.THEIR_GOAL;
            shooting = false;
        }
        controlState = ShooterControlState.BLIND_AIM;
        setVision(true);
    }

    protected void update() {
        switch (controlState) {
            case BLIND_AIM:
                updateBlind();
                break;
            case VISION_TRACKING:
                updateVision();
                break;
            case DISABLED:
                break;
        }
    }

    private void updateBlind() {
        targetAngle = Geometry.angleBetweenDegrees(robotLoc, targetPose);
        targetDistance = Geometry.distance(robotLoc, targetPose);
        readyToFire = false;
        if (Utils.withinThreshold(turret.getMeasurement(), targetAngle, LIMELIGHT_FOV/2.0)
            && targets.getDouble(0.0) == 1) {
                controlState = ShooterControlState.VISION_TRACKING;
                updateVision();
            }
        else {
            setAngle(targetAngle);
            setDistance(targetDistance);
        }
    }

    private void updateVision() {
        double targetX = this.targetX.getDouble(0.0);
        targetAngle = visionEstimateAngle(targetX);
        targetDistance = visionEstimateDistance(targetY.getDouble(0.0));
        if (Utils.outsideDeadband(targetAngle, 0.0, INNER_GOAL_AIM_THRESHOLD))
            innerPortOffset = 0.0;
        else innerPortOffset = targetAngle * 0.2; // maybe this should be a sin func?
        setAngle(targetAngle - innerPortOffset);
        setDistance(targetDistance);
        readyToFire = Utils.withinThreshold(targetX, 0.0, AIM_THRESHOLD); // TODO check hood and flywheel
        // Decide when to localize odometry if there is a vision target
        // Might be a time for filters!
    }

    /** This should be called periodically when balls are wanted to be launched
     *  It will handle whether the balls are ready to fire
     */
    private void fire() {
        if (!shooting) return;
        if (!readyToFire) return;
        feeder.runFeeder(1.0);
        // time.sleep(1); // TODO this is a placeholder, pretend it spindexes x rotations
        disable();
    }

    /**
     * Called whenever robot is disabled and after all balls are shot
     */
    public void disable() {
        controlState = ShooterControlState.DISABLED;
        setVision(false);
        shooting = false;
        fullyAimed = false;
        readyToFire = false;
        turret.disable();
        hood.disable();
        flywheel.disable();
    }

    public double visionEstimateDistance(double targetY) {
        double realElevation = targetY + LIMELIGHT_ELEVATION_OFFSET;
        return TARGET_RELATIVE_HEIGHT / Math.sin(Math.toRadians(realElevation));
    }

    private double visionEstimateAngle(double targetX) {
        return targetX + robotLoc.heading + turret.getMeasurement();
    }

    /**
     * Set the turret pid setpoint, adjusted for robot rotation
     *
     * @param angle - in global coordinations
     */
    public void setAngle(double angle) {
        turret.setSetpoint(Geometry.limitAngleDegrees(angle - robotLoc.heading));
    }

    /**
     * Controls the flywheel and hood
     * TODO make this do something smart
     * @param angle
     */
    public void setDistance(double distance) {
        flywheel.setSpeed(6000.0);
        hood.setSetpoint(0.0);
    }

    /**
     * Sets limelight leds and camera mode
     */
    private void setVision(boolean on) {
        NetworkTable lm =NetworkTableInstance.getDefault().getTable("limelight");
        if (on) {
            lm.getEntry("ledMode").setNumber(3);
            lm.getEntry("stream").setNumber(1);
        } else {
            lm.getEntry("ledMode").setNumber(1);
            lm.getEntry("stream").setNumber(2);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        flywheel.monitor();
        turret.monitor();
        hood.monitor();
    }

}
