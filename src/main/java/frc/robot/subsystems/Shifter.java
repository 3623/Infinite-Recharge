/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.EnterLowGear;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Shifter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Solenoid shifter_Piston;

  public Shifter(){
    shifter_Piston = new Solenoid(RobotMap.SHIFTER_SOLENOID);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new EnterLowGear());
  }

  public void lowGear(){
    shifter_Piston.set(false);
  }

  public void highGear(){
    shifter_Piston.set(true);
  }

  public Boolean shifterStatus(){
    return shifter_Piston.get();
  }
}
