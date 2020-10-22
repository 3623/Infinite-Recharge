/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.Drivetrain;

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
    private Drivetrain drivetrain;
    // private Intake intake;
    // private Shooter shooter;
    // private Spinner spinner;

    AnalogInput transducer = new AnalogInput(0);

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
        drivetrain = new Drivetrain();
        // intake = new Intake();
        // shooter = new Shooter();
        // spinner = new Spinner();
        // climber = new Climber();

        // Set up Port Forwarding so we can access Limelight over USB tether to robot.
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5805, "limelight.local", 5805);

        setControls();
    }

    private void setControls() {
        drivetrain.setDefaultCommand(
        new DriverControl(drivetrain, () -> driver.getY(Hand.kLeft), () -> driver.getX(Hand.kRight)));

        // intake.setDefaultCommand(
        //     new RunCommand(() -> intake.setIntaking(operator.getTriggerAxis(Hand.kRight) > 0.3), intake));

        // shooter.feeder.setDefaultCommand(
        //     new RunCommand(() -> shooter.feeder.runFeeder(operator.getTriggerAxis(Hand.kLeft) / 2), shooter.feeder));

        // shooter.hood.setDefaultCommand(
        //     new RunCommand(() -> shooter.hood.setRelative(3.0 * -operator.getY(Hand.kRight)), shooter.hood));

        // shooter.turret.setDefaultCommand(
        //     new RunCommand(() -> shooter.turret.setRelative(8.0 * operator.getX(Hand.kLeft)), shooter.turret));

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
        // mainPressure.setDouble(250 * (transducer.getVoltage() / 5) - 25);
    }

    /**
    * This function is called once each time the robot enters Disabled mode.
    */
    @Override
    public void disabledInit() {
        //drivetrain.disable();
        // shooter.disable();
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
        // shooter.turret.zero();
        // shooter.hood.zero();
        drivetrain.zeroSensors();

        // shooter.hood.enable();
        // shooter.turret.enable();

        drivetrain.setShiftMode(false);

        // shooter.setLimelightLED(false);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2); // USB Camera big,
        // Limelight Output small

        m_autonomousCommand = new TestDrive(drivetrain);

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
        //flywheelRPMAccum = 0;
        // shooter.turret.enable();
        // shooter.hood.enable();
        // shooter.setLimelightLED(false);
        // USB Camera big, Limelight Output small
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    }

    /**
    * This function is called periodically during operator control.
    */
    @Override
    public void teleopPeriodic() {
        // if (operator.getYButtonPressed()) {
        //   new SequentialCommandGroup(new PreAim(shooter, () -> /*drivetrain.model.center.heading*/ 0),
        //       new VisionAim(shooter, () -> /*drivetrain.model.center.heading*/ 0, () -> operator.getYButton())).schedule();
        // }

        if (driver.getBumperPressed(Hand.kRight)) {
            drivetrain.setShiftMode(false);
        } else if (driver.getBumperPressed(Hand.kLeft)) {
            drivetrain.setShiftMode(true);
        }

        if (driver.getStartButtonPressed()) {
            drivetrain.zeroSensors();
        }

        // // Shooter PID Setup Confirmation
        // if(operator.getPOV() == 0 && POVDebounce == false){
        //     flywheelRPMAccum += flywheelIncreaseValue;
        //     POVDebounce = true;
        // } else if(operator.getPOV() == 180 && POVDebounce == false){
        //     flywheelRPMAccum -= flywheelIncreaseValue;
        //     POVDebounce = true;
        // } else if (operator.getPOV() == -1 && POVDebounce == true){
        //     POVDebounce = false;
        // }

        // if (operator.getAButtonPressed()){
        //     System.out.println("Attempting to Do the Thing");
        //     if (shooter.flywheel.getRunning()){
        //         shooter.flywheel.setSpeed(0);
        //         System.out.println("Shooter was running. Spinning Down");
        //     } else {
        //         shooter.flywheel.setSpeed(flywheelRPMAccum);
        //         System.out.println("Shooter was not spinning. Revving Up to " + flywheelRPMAccum);
        //     }
        // }
        // if (operator.getXButtonPressed()){
        //   if (!shooter.getLimelightLEDMode()){
        //         shooter.setLimelightLED(true);
        //     } else {
        //         shooter.setLimelightLED(false);
        //     }
        // }

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        // shooter.flywheel.setSpeed(10000.0);
        // shooter.turret.enable();
        // shooter.hood.enable();
        drivetrain.runTests();
    }

    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
    }



    //

}
