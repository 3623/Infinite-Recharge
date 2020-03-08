/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Climber extends SubsystemBase {
  private Solenoid PTOLeft, PTORight;


  public Climber() {
    PTOLeft = new Solenoid(Constants.Climber.CLIMBER_PTO_SOLENOID_LEFT);
    PTORight = new Solenoid(Constants.Climber.CLIMBER_PTO_SOLENOID_RIGHT);

  }


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

  public boolean PTOLeftStatus() {
    return PTOLeft.get();
  }

  public boolean PTORightStatus() {
    return PTORight.get();
  }

}
