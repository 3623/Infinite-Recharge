/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autononmous;
import frc.robot.commands.DriverControl;
import frc.robot.commands.shooter.PreAim;
import frc.robot.commands.shooter.VisionAim;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private XboxController driver;
  private XboxController operator;
  // private Climber climber;
  //private Drivetrain drivetrain;
  //private Intake intake;
  //private Shifter shifter;
  private Shooter shooter;
  // private Spinner spinner;
  private double flywheelRPMAccum = 0;
  private double flywheelIncreaseValue = 200;
  private boolean POVDebounce;

  public final ShuffleboardTab preMatchTab = Shuffleboard.getTab("Pre-Match");
  public final ShuffleboardTab AutonomousTelemetry = Shuffleboard.getTab("Auto Telemetry");
  public final ShuffleboardTab MatchScreen = Shuffleboard.getTab("In-Match");

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    driver = new XboxController(Constants.IO.DRIVER_CONTROLLER);
    operator = new XboxController(Constants.IO.OPERATOR_CONTROLLER);
    //drivetrain = new Drivetrain();
   // intake = new Intake();
    //shifter = new Shifter();
    shooter = new Shooter();
    // spinner = new Spinner();
    // climber = new Climber();

    setControls();

    Shuffleboard.selectTab("Pre-Match");
  }

  private void setControls() {
    //drivetrain.setDefaultCommand(
    //    new DriverControl(drivetrain, () -> driver.getY(Hand.kLeft), () -> driver.getX(Hand.kRight)));

    //intake.setDefaultCommand(
    //    new RunCommand(() -> intake.setIntaking(operator.getTriggerAxis(Hand.kRight) > 0.3), intake));

    shooter.elevator.setDefaultCommand(
        new RunCommand(() -> shooter.elevator.runElevator(operator.getTriggerAxis(Hand.kLeft) / 2), shooter.elevator));

    shooter.hood.setDefaultCommand(
        new RunCommand(() -> shooter.hood.setRelative(1.5 * operator.getY(Hand.kRight)), shooter.hood));

    shooter.turret.setDefaultCommand(
        new RunCommand(() -> shooter.turret.setRelative(4.0 * operator.getX(Hand.kLeft)), shooter.turret));

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    //drivetrain.disable();
    shooter.disable();

    Shuffleboard.stopRecording();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    shooter.turret.zero();
    shooter.hood.zero();
    //drivetrain.zeroSensors();

    shooter.hood.enable();
    shooter.turret.enable();

    //shifter.lowGear();

    Shuffleboard.selectTab("Auto Telemetry");
    shooter.setLimelightLED(false);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2); // USB Camera big,
                                                                                             // Limelight Output small
    Shuffleboard.startRecording();

    //m_autonomousCommand = new Autononmous(drivetrain, 0.5, 2.0);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    flywheelRPMAccum = 0;
    shooter.turret.enable();
    shooter.hood.enable();
    Shuffleboard.selectTab("In-Match");
    shooter.setLimelightLED(false);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2); // USB Camera big,
                                                                                             // Limelight Output small
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (operator.getYButtonPressed()) {
      new SequentialCommandGroup(new PreAim(shooter, () -> /*drivetrain.model.center.heading*/ 0),
          new VisionAim(shooter, () -> /*drivetrain.model.center.heading*/ 0, () -> operator.getYButton())).schedule();
    }

    //if (driver.getBumperPressed(Hand.kRight)) {
   //   shifter.lowGear();
   // } else if (driver.getBumperPressed(Hand.kLeft)) {
    //  shifter.highGear();
    //}

    //if (driver.getStartButtonPressed()) {
    //  drivetrain.zeroSensors();
    //}

    if(operator.getPOV() == 0 && POVDebounce == false){
      flywheelRPMAccum += flywheelIncreaseValue;
      POVDebounce = true;
    }
    else if(operator.getPOV() == 180 && POVDebounce == false){
      flywheelRPMAccum -= flywheelIncreaseValue;
      POVDebounce = true;
    }
    else if (operator.getPOV() == -1 && POVDebounce == true){
      POVDebounce = false;
    }

    SmartDashboard.putNumber("RPM Set", flywheelRPMAccum);
    SmartDashboard.putBoolean("Is Shooter Running", shooter.flywheel.getRunning());

    if (operator.getAButtonPressed()){
      if (shooter.flywheel.getRunning()){
        shooter.flywheel.setSpeed(0);
      }
      else {
        shooter.flywheel.setSpeed(flywheelRPMAccum);
      }
    }

    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //shooter.flywheel.setSpeed(10000.0);
    shooter.turret.enable();
    shooter.hood.enable();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }

  // TODO pressure stuff

  // private ShuffleboardTab shuffle = Shuffleboard.getTab("SmartDashboard");

  // AnalogInput transducer = new AnalogInput(0);

  // NetworkTableEntry mainPressure = shuffle.add("Main System Pressure",
  // 0).withWidget(BuiltInWidgets.kDial)
  // .withProperties(Map.of("min", 0, "max", 130)).getEntry();

  // mainPressure.setDouble(250 * (transducer.getVoltage() / 5) - 25);

}
