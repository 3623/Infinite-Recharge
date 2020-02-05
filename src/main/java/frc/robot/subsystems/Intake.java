/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Intake extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private VictorSPX collector;
  private TalonSRX indexer;

  private Solenoid collectorDrop;
  // private VictorSPX collectorDropper;

  private DigitalInput ballSensor1, ballSensor2, ballSensor3, ballSensor4, ballSensor5;

  public Intake(){
    collector = new VictorSPX(IntakeConstants.INTAKE_COLLECTOR_MOTOR_SPX);
    indexer = new TalonSRX(IntakeConstants.INTAKE_INDEXER_MOTOR_SRX);

    collectorDrop = new Solenoid(IntakeConstants.INTAKE_DROP_SOLENOID);
    // collectorDropper = new VictorSPX(IntakeConstants.INTAKE_DROP_MOTOR_SPX);
  }

  public void dropCollector(){
    collectorDrop.set(true);
  }

  public void raiseCollector(){
    collectorDrop.set(false);
  }

  public void runCollector(double collectorSpeed){
    collector.set(ControlMode.PercentOutput, collectorSpeed);
  }

  public void stopCollector(){
    collector.set(ControlMode.PercentOutput, 0.0);
  }

  public void runIndexer(double indexerSpeed){
    indexer.set(ControlMode.PercentOutput, indexerSpeed);
  }

  public void stopIndexer(){
    indexer.set(ControlMode.PercentOutput, 0.0);
  }

}
