/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriverControl extends CommandBase {
    private final Drivetrain Drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    public DriverControl(Drivetrain DT, DoubleSupplier forward, DoubleSupplier rotation) {
        Drive = DT;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(Drive);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double joystickY = -m_forward.getAsDouble();
        double joystickR = m_rotation.getAsDouble();
        Boolean quickTurn;
        if (Math.abs(joystickY) < 0.5)
            quickTurn = true;
        else
            quickTurn = false;

        /*
         * if (m_seekTarget.getAsBoolean() && tv.getDouble(0.0) > 0) { double x =
         * tx.getDouble(0.0); double headingError = -tx.getDouble(0.0); double
         * steeringAdjust = 0.0; if (x > 1.0) { steeringAdjust = kP * headingError -
         * minCommand; } else if (x < -1.0) { steeringAdjust = kP * headingError +
         * minCommand; }
         *
         * Drive.terribleDrive(joystickY, joystickR + steeringAdjust, true); } else
         */ if (quickTurn) {
            Drive.terribleDrive(joystickY * 0.5, joystickR, quickTurn);
        } else {
            Drive.terribleDrive(joystickY * Math.abs(joystickY), joystickR * 0.5, quickTurn);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}