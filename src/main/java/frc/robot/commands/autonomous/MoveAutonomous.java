/*/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class Autononmous extends ParallelRaceGroup {

public class MoveAutonomous extends ParallelRaceGroup {

    public MoveAutonomous(Drivetrain drive, double speed, double time) {
        addCommands(new WaitCommand(time), new StartEndCommand(() -> drive.terribleDrive(speed, 0.0, false),
                () -> drive.terribleDrive(0.0, 0.0, false), drive));
    }

}