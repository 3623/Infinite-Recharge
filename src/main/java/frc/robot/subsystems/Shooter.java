/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.util.Utils;

public class Shooter extends TerribleSubsystem {
  protected final int UPDATE_RATE = 70;

  public Turret turret;
  public Feeder feeder;
  public Hood hood;
  public Flywheel flywheel;

  private static final double AIM_THRESHOLD = 2.0;

  private boolean LEDStatus = false;

  public boolean targetAcquired = false;
  public boolean aimed = false;
  public boolean atSpeed = false;
  public boolean readyToFire = false;

  NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)
  NetworkTableEntry ty = Lime.getEntry("ty"); // Vertical Offset From Crosshair to Target (-20.5 to 20.5 degrees)
  NetworkTableEntry ta = Lime.getEntry("ta"); // Target Area (0% of Image to 100% of Image)
  NetworkTableEntry tv = Lime.getEntry("tv"); // Valid Targets (0 or 1)

  private final double LIMELIGHT_ELEVATION_OFFSET = 20.0;
  private final double TARGET_RELATIVE_HEIGHT = 2.0; // meters
  private final double TARGET_WIDTH = 1.0; // m

  public double x, y, area, valid;

  public Shooter() {
    turret = new Turret();
    hood = new Hood();
    feeder = new Feeder();
    flywheel = new Flywheel();

    this.updateThreadStart();
  }

  protected void update() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    valid = tv.getDouble(0.0);

    if (valid > 0.0){
      targetAcquired = true;
    }
    else{
      targetAcquired = false;
    }
    aimed = isAimed(x) && targetAcquired;
    atSpeed = flywheel.isAtSpeed();
    readyToFire = aimed && atSpeed;

    currentRPM.setNumber(flywheel.getVelocity());
    IsAimed.setBoolean(aimed);
    flywheelRunning.setBoolean(flywheel.getRunning());

    flywheel.monitor();
    turret.monitor();
    hood.monitor();
  }

  private Boolean isAimed(double offset) {
    return Utils.withinThreshold(x, 0.0, AIM_THRESHOLD);
  }

  public void setLimelightLED(Boolean on) {
    if (on) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode")
          .setNumber(Constants.Shooter.LIMELIGHT_LED_FORCE_ON);
      LEDStatus = true;
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode")
          .setNumber(Constants.Shooter.LIMELIGHT_LED_FORCE_OFF);
      LEDStatus = false;
    }
  }

  public void setCameraMode(Boolean vision) {
    if (vision) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    }
  }

  public void disable() {
    turret.disable();
    hood.disable();
    flywheel.disable();
  }

  public double getDistance(double targetY) {
    double realElevation = targetY + LIMELIGHT_ELEVATION_OFFSET;
    return TARGET_RELATIVE_HEIGHT / Math.sin(Math.toRadians(realElevation));
  }
}
