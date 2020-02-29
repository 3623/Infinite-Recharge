/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.node.BooleanNode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Climber extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Solenoid PTOLeft, PTORight;
  private Solenoid climberLockLeft, climberLockRight;

  public Climber() {
    PTOLeft = new Solenoid(ClimberConstants.CLIMBER_PTO_SOLENOID_LEFT);
    PTORight = new Solenoid(ClimberConstants.CLIMBER_PTO_SOLENOID_RIGHT);
    // climberLockLeft = new Solenoid(ClimberConstants.CLIMBER_LOCK_SOLENOID_LEFT);
    // climberLockRight = new
    // Solenoid(ClimberConstants.CLIMBER_LOCK_SOLENOID_RIGHT);
  }

  // TODO fix this later
  // public void lock(){
  // climberLockLeft.set(true);
  // climberLockRight.set(true);
  // }

  // public void releaseLeft(){
  // climberLockLeft.set(false);
  // }

  // public void releaseRight(){
  // climberLockRight.set(false);
  // }

  public void engagePTO() {
    PTOLeft.set(true);
    PTORight.set(true);
  }

  public void disengagePTOLeft() {
    PTOLeft.set(false);
  }

  public void disengagePTORight() {
    PTORight.set(false);
  }

  // public boolean climberLockLeftStatus(){
  // return climberLockLeft.get();
  // }

  // public boolean climberLockRightStatus(){
  // return climberLockRight.get();
  // }

  public boolean PTOLeftStatus() {
    return PTOLeft.get();
  }

  public boolean PTORightStatus() {
    return PTORight.get();
  }

}
