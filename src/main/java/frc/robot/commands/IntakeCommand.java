package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends SequentialCommandGroup{

    public IntakeCommand(Intake intake, Spindexer spindexer, BooleanSupplier stop) {
        addCommands((new IntakeActive(intake, spindexer)).withInterrupt(stop),
                    (new StartEndCommand(() -> spindexer.setIndexing(true),
                                         () -> spindexer.setIndexing(false),
                                         spindexer)).withTimeout(Spindexer.INDEX_TIME));
    }

    public IntakeCommand(Intake intake, Spindexer spindexer, double time) {
        addCommands((new IntakeActive(intake, spindexer)).withTimeout(time),
                    (new StartEndCommand(() -> spindexer.setIndexing(true),
                                         () -> spindexer.setIndexing(false),
                                         spindexer)).withTimeout(Spindexer.INDEX_TIME));
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
            intake.setIntaking(true);
            spindexer.setIndexing(true);
        }

        @Override
        public void end(boolean interrupted) {
            intake.setIntaking(false);
        }
    }

}
