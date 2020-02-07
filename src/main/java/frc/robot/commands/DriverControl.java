/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriverControl extends CommandBase {
    private final Drivetrain Drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;
    private final BooleanSupplier m_seekTarget;
    NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight"); // The Limelight Vision system posts several useful bits
                                                                                    // of data to Network Tables.
    NetworkTableEntry tx = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)

    private final double kP = -0.1; // proportional control for target seeking
    private final double minCommand = 0.05; // apply minimum to make bot move as output approaches 0; 


  public DriverControl(Drivetrain DT, DoubleSupplier forward, DoubleSupplier rotation, BooleanSupplier seekTarget) {
    Drive = DT;
    m_forward = forward;
    m_rotation = rotation;
    m_seekTarget = seekTarget;
    addRequirements(Drive);
    }

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        Boolean quickTurn;
        if (Math.abs(m_forward.getAsDouble()) < 0.5)
            quickTurn = true;
        else
            quickTurn = false;
        // quickTurn = true;

        if (m_seekTarget.getAsBoolean()){
            double x = tx.getDouble(0.0);
            double headingError = -tx.getDouble(0.0);
            double steeringAdjust = 0.0;
            double rightOutput = m_forward.getAsDouble() - m_rotation.getAsDouble();
            double leftOutput = m_forward.getAsDouble() + m_rotation.getAsDouble();

            if (x > 1.0){
                steeringAdjust = kP*headingError - minCommand;
            }
            else if (x < 1.0){
                steeringAdjust = kP*headingError + minCommand;
            }
            leftOutput += steeringAdjust;
            rightOutput -= steeringAdjust;
            Drive.setOpenLoop(leftOutput, rightOutput);
        }
        else if (quickTurn) {
            Drive.driverControl(-m_forward.getAsDouble() * 0.5, m_rotation.getAsDouble(), quickTurn);
        } else {
            Drive.driverControl(-m_forward.getAsDouble() * Math.abs(m_forward.getAsDouble()), m_rotation.getAsDouble() * 0.5, quickTurn);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}