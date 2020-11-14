/*/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.modeling.FieldPositions;
import frc.robot.subsystems.Drivetrain;

public class TestDrive extends CommandBase {

    private final Drivetrain dt;

    public TestDrive(Drivetrain drive) {
        dt = drive;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        dt.waypointNav.clearWaypoints();
        dt.zeroSensors();
        // dt.model.setPosition(0.0, 0.0, 0.0);
        // dt.waypointNav.addWaypoint(new Waypoint(0.0, 1.0, 0.0, 0.3, true));
        // dt.startPathFollowing();

        // // Right
		// dt.model.setPosition(FieldPositions.RIGHT_START);
		// dt.waypointNav.addWaypoint(FieldPositions.RIGHT1);
		// dt.waypointNav.addWaypoint(FieldPositions.RIGHT2);
		// dt.waypointNav.addWaypoint(FieldPositions.RIGHT3);
		// dt.waypointNav.addWaypoint(FieldPositions.RIGHT4);
        // dt.waypointNav.addWaypoint(FieldPositions.RIGHT5);
        // dt.startPathFollowing();

        // Steal balls
		dt.model.setPosition(FieldPositions.STEAL_START);
		dt.waypointNav.addWaypoint(FieldPositions.STEAL1);
		dt.waypointNav.addWaypoint(FieldPositions.STEAL2);
		dt.waypointNav.addWaypoint(FieldPositions.STEAL3);
		dt.waypointNav.addWaypoint(FieldPositions.STEAL4);
        dt.waypointNav.addWaypoint(FieldPositions.STEAL5);
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