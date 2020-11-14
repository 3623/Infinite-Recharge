package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends SequentialCommandGroup{

    public IntakeCommand(BooleanSupplier stop, Intake intake, Spindexer spindexer) {
        addCommands((new IntakeActive(intake, spindexer)).withInterrupt(stop),
                    (new IndexBalls(spindexer)).withTimeout(Spindexer.INDEX_TIME));
    }


    private class IntakeActive extends CommandBase {
        private Intake intake;
        private Spindexer spindexer;

        private IntakeActive(Intake intake, Spindexer spindexer) {
            this.intake = intake;
            this.spindexer = spindexer;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            super.initialize();
            intake.setIntaking(true);
            spindexer.setIndexing(true);
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            intake.setIntaking(false);
        }
    }

}
