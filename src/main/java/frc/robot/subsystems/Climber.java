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
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Climber extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Solenoid PTO;
  private Solenoid climberLock;
  

  public Climber(){
    PTO = new Solenoid(ClimberConstants.CLIMBER_PTO_SOLENOID);
    climberLock = new Solenoid(ClimberConstants.CLIMBER_LOCK_SOLENOID);
  }

  public void lock(){
    climberLock.set(true);
  }

  public void release(){
    climberLock.set(false);
  }

  public void engagePTO(){
    PTO.set(true);
  }

  public void disengagePTO(){
    PTO.set(false);
  }

  public boolean climberLockStatus(){
    return climberLock.get();
  }

  public boolean PTOStatus(){
    return PTO.get();
  }

}
