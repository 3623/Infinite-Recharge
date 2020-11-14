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
import frc.robot.subsystems.Shooter;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private XboxController driver;
    private XboxController operator;
    // private Climber climber;
    private Drivetrain drivetrain;
    // private Intake intake;
    private Shooter shooter;

    AnalogInput transducer = new AnalogInput(0);


    @Override
    public void robotInit() {
        driver = new XboxController(Constants.IO.DRIVER_CONTROLLER);
        operator = new XboxController(Constants.IO.OPERATOR_CONTROLLER);
        drivetrain = new Drivetrain();
        shooter = new Shooter(drivetrain.model.center);
        // intake = new Intake();
        // climber = new Climber();

        // Set up Port Forwarding so we can access Limelight over USB tether to robot.
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5805, "limelight.local", 5805);

        drivetrain.setDefaultCommand(
        new DriverControl(drivetrain, () -> driver.getY(Hand.kLeft), () -> driver.getX(Hand.kRight)));

        // intake.setDefaultCommand(
        //     new RunCommand(() -> intake.setIntaking(operator.getTriggerAxis(Hand.kRight) > 0.3), intake));
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // mainPressure.setDouble(250 * (transducer.getVoltage() / 5) - 25);
    }


    @Override
    public void disabledInit() {
        drivetrain.disable();
        shooter.disable();
    }


    @Override
    public void disabledPeriodic() {
    }


    @Override
    public void autonomousInit() {
        drivetrain.zeroSensors();
        shooter.zeroSensors();

        drivetrain.setShiftMode(true);

        // shooter.setLimelightLED(false);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2); // USB Camera big,
        // Limelight Output small

        // m_autonomousCommand = new TestDrive(drivetrain);

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }


    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        // stop running autonomous command
        if (m_autonomousCommand != null) m_autonomousCommand.cancel();
        drivetrain.setShiftMode(true);
    }


    @Override
    public void teleopPeriodic() {
        // if (operator.getYButtonPressed()) {}

        if (driver.getBumperPressed(Hand.kRight)) {
            drivetrain.setShiftMode(false);
        } else if (driver.getBumperPressed(Hand.kLeft)) {
            drivetrain.setShiftMode(true);
        }

        if (driver.getStartButtonPressed()) {
            drivetrain.zeroSensors();
        }
    }


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        // drivetrain.runTests();
    }


    @Override
    public void testPeriodic() {
        shooter.setAngle(Math.toDegrees(Math.atan2(operator.getRawAxis(1), operator.getRawAxis(0))));
    }
}
