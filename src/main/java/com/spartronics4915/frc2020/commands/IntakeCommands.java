package com.spartronics4915.frc2020.commands;

import java.util.Set;

import com.spartronics4915.frc2020.subsystems.Indexer;
import com.spartronics4915.frc2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeCommands
{
    private final Intake mIntake;
    private final Indexer mIndexer;

    public IntakeCommands(Intake intake, Indexer indexer)
    {
        mIntake = intake;
        mIndexer = indexer;
        mIntake.setDefaultCommand(new Stop());
    }

    /**
     * A {@link StartEndCommand} allows us to specify an execute() and end()
     * condition, and runs until interrupted.
     *
     * @param Runnable onInit Runs over and over until interrupted
     * @param Runnable onEnd (boolean interrupted) Method to call once when ended
     * @param Subsystem requirement For both the CommandScheduler and the above method references.
     */

    /**
     * This {@link StartEndCommand} harvests balls by running {@link Intake}.intake continuously,
     * unless terminated by a second press of the Harvest button.
     */
    public class Harvest extends CommandBase
    {
        @Override
        public void initialize()
        {
            mIntake.harvest();
        }

        @Override
        public boolean isFinished()
        {
            return mIndexer.getIntakeBallLoaded();
        }

        @Override
        public void end(boolean interrupted)
        {
            mIntake.stop();
        }

        @Override
        public Set<Subsystem> getRequirements()
        {
            return Set.of(mIntake);
        }
    }

    /**
     * This {@link StartEndCommand} runs the intake motor backwards by calling
     * {@link Intake}.reverse repeatedly.
     * <p>
     * Note that this is not an Unjam command. The {@link Intake} subsystem only
     * controls the mechanical vector roller.
     */
    public class Eject extends StartEndCommand // TODO: Does this execute(), or initialize()?
    {
        public Eject()
        {
            super(mIntake::reverse, mIntake::stop, mIntake);
        }
    }

    /**
     * This {@link RunCommand} stops all intake motors by simply calling
     * Intake.stop repeatedly.
     * <p>
     * It also happens to be our default command.
     */
    public class Stop extends RunCommand
    {
        public Stop()
        {
            super(mIntake::stop, mIntake);
        }
    }
}
