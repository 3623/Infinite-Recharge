/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Feeder extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX feederSRX;

  // private DigitalInput ballSensor1, ballSensor2, ballSensor3, ballSensor4,
  // ballSensor5;

  public Feeder() {
    feederSRX = new WPI_TalonSRX(Constants.Shooter.FEEDER_MOTOR_SRX);
  }

  public void runFeeder(double feederSpeed) {
    feederSRX.set(ControlMode.PercentOutput, feederSpeed);
  }

  public void stopFeeder() {
    feederSRX.set(ControlMode.PercentOutput, 0.0);
  }

}
