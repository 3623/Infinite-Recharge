/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;

public class FeedForTime extends ParallelRaceGroup {

    public FeedForTime(Feeder feeder, double speed, double time) {
        addCommands(new WaitCommand(time),
                new StartEndCommand(() -> feeder.runFeeder(speed), () -> feeder.stopFeeder(), feeder));
    }
}
