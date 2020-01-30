/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.IOConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController driver = new XboxController(IOConstants.DRIVER_CONTROLLER);
  private final XboxController operator = new XboxController(IOConstants.OPERATOR_CONTROLLER);
  private final Climber climb = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shifter shifter = new Shifter();
  private final Shooter shooter = new Shooter();
  private final Spinner spinner = new Spinner();


  private JoystickButton driverA, driverB, driverX, driverY, driverLB, driverRB, driverStart, driverBack, driverL3, driverR3,
                          operatorA, operatorB, operatorX, operatorY, operatorLB, operatorRB, operatorStart, operatorBack, operatorL3, operatorR3;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings'
    drivetrain.setDefaultCommand(
      new DriverControl(
        drivetrain, 
        () -> driver.getY(Hand.kLeft), 
        () -> driver.getY(Hand.kRight)));
      configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverRB = new JoystickButton(driver, Button.kBumperRight.value);
    driverRB.whenPressed(new ConditionalCommand(new InstantCommand(shifter::lowGear, shifter), new InstantCommand(shifter::highGear, shifter),shifter::shifterStatus));

    driverX = new JoystickButton(driver, Button.kX.value);
    driverX.whenPressed(new InstantCommand(drivetrain::zeroSensors,drivetrain));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
