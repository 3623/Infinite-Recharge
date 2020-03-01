/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class VisionAim extends CommandBase {
    private static final double FOCUS_POINT_SCALING_FACTOR = 0.05;
    private static Shooter shooterSystem;
    private static DoubleSupplier robotHeading;
    private Boolean hold = false;

    public VisionAim(Shooter shooter, DoubleSupplier heading, Boolean hold) {
        shooterSystem = shooter;
        robotHeading = heading;
        this.hold = hold;
        addRequirements(shooter);
    }

    // Initialize is called immediately when the command is scheduled.
    @Override
    public void initialize() {
        shooterSystem.setLimelightLED(true);
    }

    public void execute() {
        double degreesFromGlobalForward = robotHeading.getAsDouble() + shooterSystem.turret.getAngle();
        double trigFactor = -degreesFromGlobalForward * FOCUS_POINT_SCALING_FACTOR;
        shooterSystem.turret.setAngle(-shooterSystem.x + trigFactor + shooterSystem.turret.getAngle());
    }

    public boolean isFinished() {
        return (shooterSystem.targetAcquired && !hold);
    }
    /*if abs(limelightentries.tx) > 1.5 {
        runWith
    }*/
}