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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.*;
import frc.robot.commands.shooter.PreAim;
import frc.robot.commands.shooter.VisionAim;
import frc.robot.subsystems.*;
import frc.robot.Constants.IOConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
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
  private final Elevator elevator = new Elevator();

  public final ShuffleboardTab preMatchTab = Shuffleboard.getTab("Pre-Match");
  public final ShuffleboardTab AutonomousTelemetry = Shuffleboard.getTab("Auto Telemetry");
  public final ShuffleboardTab MatchScreen = Shuffleboard.getTab("In-Match");

  private JoystickButton driverA, driverB, driverX, driverY, driverLB, driverRB, driverStart, driverBack, driverL3,
      driverR3, operatorA, operatorB, operatorX, operatorY, operatorLB, operatorRB, operatorStart, operatorBack,
      operatorL3, operatorR3;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    intake.setDefaultCommand(new RunCommand(() -> intake.setIntaking(false), intake));
    elevator.setDefaultCommand(
        new RunCommand(() -> elevator.runElevator(operator.getTriggerAxis(Hand.kLeft) / 2), elevator));

    shooter.hood.enable();
    shooter.turret.enable();

    /*shooter.hood.setDefaultCommand(new RunCommand(() -> shooter.hood.setRelative(1.5 * operator.getY(Hand.kRight)), shooter.hood));
    shooter.turret.setDefaultCommand(new RunCommand(() -> shooter.turret.setOffset(4.0 * operator.getX(Hand.kLeft)), shooter.turret));
    */

    shooter.hood.setDefaultCommand(new RunCommand(() -> shooter.hood.runWithOutput(0.2 * operator.getY(Hand.kRight)), shooter.hood));
    shooter.turret.setDefaultCommand(new RunCommand(() -> shooter.turret.runwithOutput(.4 * operator.getX(Hand.kLeft)), shooter.turret));


    shooter.setDefaultCommand(new RunCommand(() -> shooter.runShooterPID(0), shooter));

    drivetrain.setDefaultCommand(
        new DriverControl(drivetrain, () -> driver.getY(Hand.kLeft), () -> driver.getX(Hand.kRight)));
    // Configure the button bindings, tying button presses to commands.
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Shifter Commands. Inline Command Declaration.
    driverRB = new JoystickButton(driver, Button.kBumperRight.value);
    driverRB.whenPressed(new InstantCommand(shifter::lowGear, shifter));
    driverLB = new JoystickButton(driver, Button.kBumperLeft.value);
    driverLB.whenPressed(new InstantCommand(shifter::highGear, shifter));

    // Zero Sensors. Inline Command Declaration.
    driverX = new JoystickButton(driver, Button.kX.value);
    driverX.whenPressed(new InstantCommand(drivetrain::zeroSensors, drivetrain));

    // Drop Intake. On Driver's Control
    driverA = new JoystickButton(driver, Button.kA.value);
    driverA.whileHeld(new RunCommand(() -> intake.setIntaking(true)));

    // Shooter Command
    operatorY = new JoystickButton(operator, Button.kY.value);
    operatorY.whenHeld(new SequentialCommandGroup(new PreAim(shooter, () -> drivetrain.model.center.heading),
        new VisionAim(shooter, () -> drivetrain.model.center.heading, true)));

    operatorX = new JoystickButton(operator, Button.kX.value);
    operatorX.whileHeld(new spitBallsOut(intake, elevator));

    operatorA = new JoystickButton(operator, Button.kA.value);
    //operatorA.toggleWhenPressed(new RunCommand(() -> shooter.runShooterPID(11000)),true);  
    operatorA.toggleWhenPressed(new RunCommand(() -> shooter.runShooterAccumRamp(0.8), shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new Autononmous(drivetrain, 0.5, 3);
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new WaitCommand(4),
        new RunCommand(() -> shooter.runShooterPID(20000), shooter)
        ),
      new ParallelRaceGroup(
        new WaitCommand(8),
        new RunCommand(()-> elevator.runElevator(.75), elevator)
        ),
      new Autononmous(drivetrain, .45, 3)
      );

  }
}
