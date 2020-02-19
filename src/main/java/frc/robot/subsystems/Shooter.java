/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.Turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final int UPDATE_RATE = 70;
  private CANSparkMax shooterMaster, shooterFollower;
  private CANPIDController shooterPID;
  private Turret turret;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public boolean targetAcquired;

  private ShooterState state = ShooterState.IDLE;

  private enum ShooterState {
    IDLE, REVVING_UP, READY_TO_FIRE, RETURNING_TO_IDLE,
  }

  ShuffleboardTab settings = Shuffleboard.getTab("Tuning");
  private NetworkTableEntry shooterPIDSystem = settings.addPersistent("Shooter PID System", shooterPID)
      .withWidget(BuiltInWidgets.kPIDController).getEntry();

  NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight");
  // The Limelight Vision system posts several useful bits of data to Network
  // Tables.
  NetworkTableEntry tx = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)
  NetworkTableEntry ty = Lime.getEntry("ty"); // Vertical Offset From Crosshair to Target (-20.5 to 20.5 degrees)
  NetworkTableEntry ta = Lime.getEntry("ta"); // Target Area (0% of Image to 100% of Image)
  NetworkTableEntry tv = Lime.getEntry("tv"); // Valid Targets (0 or 1, False/True)
  double x, y, area, valid;

  public Shooter() {
    shooterMaster = new CANSparkMax(1, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(2, MotorType.kBrushless);
    shooterFollower.follow(shooterMaster, true);
    shooterPID = shooterMaster.getPIDController();

    kP = 6e-5; // BANANA why is this not outside of constructor?
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setIZone(kIz);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(kMinOutput, kMaxOutput);

    turret = new Turret();

    this.updateThreadStart();
  }

  public void runShooterPID(double RPM) {
    shooterPID.setReference(RPM, ControlType.kVelocity);
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
    valid = tv.getDouble(0.0);
    this.monitor();
  }

  private void monitor() {

  }
}
