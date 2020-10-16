/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.util.Pose;

public class PreAim extends CommandBase {
    private Shooter shooter;
    private Pose loc;

    /*
     * Constructor of The Command Arguments - Spinner Subsystem
     */
    public PreAim(Shooter shooterSubsystem, Pose robotPose) {
        shooter = shooterSubsystem;
        loc = robotPose;
        addRequirements(shooter);
    }

    // Initialize is called immediately when the command is scheduled.
    public void initialize() {
        shooter.setLimelightLED(true);
    }

    public void execute() {
        shooter.turret.setAngle(-robotHeading.getAsDouble());
    }

    public boolean isFinished() {
        return shooter.targetAcquired;
    }

    public void end(boolean interrupted){
        if (!interrupted){
            System.out.println("Successfully Pre-Aimed");
        }
        else{
            System.out.println("Pre-Aim Interruped");
        }

    }
}
