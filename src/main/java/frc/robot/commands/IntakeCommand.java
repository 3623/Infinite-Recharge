package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

/**
 * A command to drive the robot with joystick input (passed in as
 * {@link DoubleSupplier}s). Written explicitly for pedagogical purposes -
 * actual code should inline a command this simple with
 * {@link edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final BooleanSupplier active;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public IntakeCommand(BooleanSupplier active, Intake subsystem) {
        intake = subsystem;
        this.active = active;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntaking(true);
    }
}