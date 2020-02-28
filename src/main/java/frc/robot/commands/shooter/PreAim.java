/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class PreAim extends CommandBase {
    private static Shooter shooterSystem;
    private static DoubleSupplier robotHeading;

    /*
     * Constructor of The Command Arguments - Spinner Subsystem
     */
    public PreAim(Shooter shooter, DoubleSupplier heading) {
        shooterSystem = shooter;
        robotHeading = heading;
        addRequirements(shooter);
    }

    // Initialize is called immediately when the command is scheduled.
    public void initialize() {
        shooterSystem.setLimelightLED(true);
    }

    public void execute() {
        shooterSystem.turret.setAngle(-robotHeading.getAsDouble());
    }

    public boolean isFinished() {
        return shooterSystem.targetAcquired;
    }
}
