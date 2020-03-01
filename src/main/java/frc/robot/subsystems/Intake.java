/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Intake extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final double INTAKE_SPEED = -0.7;
  private WPI_VictorSPX collector;

  private Solenoid collectorDrop;

  // private DigitalInput ballSensor1, ballSensor2, ballSensor3, ballSensor4,
  // ballSensor5;

  public Intake() {
    collector = new WPI_VictorSPX(Constants.Intake.INTAKE_COLLECTOR_MOTOR_SPX);

    collectorDrop = new Solenoid(Constants.Intake.INTAKE_DROP_SOLENOID);
  }

  public void setIntaking(Boolean intake) {
    if (intake) {
      collectorDrop.set(true);
      setSpeed(INTAKE_SPEED);
    } else {
      collectorDrop.set(false);
      collector.disable();
    }
  }

  public void setSpeed(double collectorSpeed) {
    collector.set(ControlMode.PercentOutput, collectorSpeed);
  }

  public boolean collectorStatus() {
    return collectorDrop.get();
  }
}
