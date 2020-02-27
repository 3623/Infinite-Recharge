/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;


public class spitBallsOut extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private static Intake intakeSystem;
  private static Elevator elevatorSystem;


  /* Constructor of The Command
      Arguments - Spinner Subsystem
  */
  public spitBallsOut(Intake intake, Elevator elevator){
    intakeSystem = intake;
    elevatorSystem = elevator;
    addRequirements(intakeSystem, elevatorSystem);
  }

  // Initialize is called immediately when the command is scheduled.
  public void initialize() {
    
    }

  public void execute(){
    intakeSystem.setSpeed(1.0);
    elevatorSystem.runElevator(1.0);
  }
    

  public boolean isFinished(){
    return false;
  }
}
