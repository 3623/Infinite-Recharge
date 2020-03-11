/*/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.controls.CubicSplineFollower.Waypoint;
import frc.robot.subsystems.Drivetrain;

public class RunOneMeter extends CommandBase {

    private final Drivetrain dt;

    public RunOneMeter(Drivetrain drive) {
        dt = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        dt.waypointNav.clearWaypoints();
        dt.zeroSensors();
        dt.model.setPosition(0.0, 0.0, 0.0);
        dt.waypointNav.addWaypoint(new Waypoint(0.5, 2.0, 45.0, 0.3, true));
        dt.startPathFollowing();
    }

    @Override
    public boolean isFinished() {
        return dt.waypointNav.isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        dt.disable();
    }
}