/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shifter;

/**
 * An example command.  You can replace me with your own command.
 */
public class EnterLowGear extends CommandBase {
  private final Shifter ShiftingGearbox;
  public EnterLowGear(final Shifter shift) {
    ShiftingGearbox = shift;
    addRequirements(ShiftingGearbox);
  }

  public void initialize(){
    ShiftingGearbox.lowGear();
  }

  public boolean isFinished(){
    return true;
  }

}
