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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Utils;

public class Shooter extends SubsystemBase {
  private final int UPDATE_RATE = 70;

  public Turret turret;
  public Elevator elevator;
  public Hood hood;
  public Flywheel flywheel;

  private static final double AIM_THRESHOLD = 2.0;

  public boolean targetAcquired = false;
  public boolean aimed = false;
  public boolean atSpeed = false;
  public boolean readyToFire = false;

  NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)
  NetworkTableEntry ty = Lime.getEntry("ty"); // Vertical Offset From Crosshair to Target (-20.5 to 20.5 degrees)
  NetworkTableEntry ta = Lime.getEntry("ta"); // Target Area (0% of Image to 100% of Image)
  NetworkTableEntry tv = Lime.getEntry("tv"); // Valid Targets (0 or 1, False/True)

  public double x, y, area;

  public Shooter() {
    turret = new Turret();
    hood = new Hood();
    elevator = new Elevator();
    flywheel = new Flywheel();

    this.updateThreadStart();
  }

  private void updateThreadStart() {
    Thread t = new Thread(() -> {
      while (!Thread.interrupted()) {
        this.update();
        try {
          Thread.sleep(1000 / UPDATE_RATE);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    });
    t.start();
  }

  private void update() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    targetAcquired = tv.getBoolean(false);
    aimed = isAimed(x) && targetAcquired;
    atSpeed = flywheel.isAtSpeed();
    readyToFire = aimed && atSpeed;

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
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode")
          .setNumber(Constants.Shooter.LIMELIGHT_LED_FORCE_OFF);
    }
  }

  public void disable() {
    turret.disable();
    hood.disable();
    flywheel.disable();
  }
}
