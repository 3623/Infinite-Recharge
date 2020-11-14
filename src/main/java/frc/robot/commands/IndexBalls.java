package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spindexer;

public class IndexBalls extends CommandBase {
    private Spindexer spindexer;

    public IndexBalls(Spindexer spindexer) {
        this.spindexer = spindexer;
    }

    @Override
    public void initialize() {
        spindexer.setIndexing(true);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.setIndexing(false);
    }

}

