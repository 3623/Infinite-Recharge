/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Elevator extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX elevatorSRX;

  private DigitalInput ballSensor1, ballSensor2, ballSensor3, ballSensor4, ballSensor5;

  public Elevator() {
    elevatorSRX = new WPI_TalonSRX(IntakeConstants.INTAKE_ELEVATOR_MOTOR_SRX);
  }

  public void runElevator(double elevatorSpeed) {
    elevatorSRX.set(ControlMode.PercentOutput, elevatorSpeed);
  }

  public void stopElevator() {
    elevatorSRX.set(ControlMode.PercentOutput, 0.0);
  }

}
