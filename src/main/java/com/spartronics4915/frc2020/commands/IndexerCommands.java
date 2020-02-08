package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Indexer;g

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class IndexerCommands
{
    /**
     * Commands with simple logic statements should be implemented as a
     * {@link FunctionalCommand}. This saves the overhead of a full
     * {@link CommandBase}, but still allows us to deal with isFinished.
     * <p>
     * A FunctionalCommand takes five inputs:
     * @param Runnable onInit
     * @param Runnable onExecute
     * @param Consumer<Boolean> onEnd (Boolean interrupted)
     * @param BooleanSupplier isFinished
     * @param Subsystem requirement For both the CommandScheduler and the above method references.
     * <p>
     * Each of these parameters corresponds with a method in the CommandBase class.
     */

    /**
     * This {@link FunctionalCommand} harvests balls by running {@link Intake}.intake continuously,
     * unless terminated by a second press of the Harvest button or
     * a positive reading from {@link Intake}.isBallHeld.
     */
    public class WaitForBallHeld extends FunctionalCommand
    {
        public WaitForBallHeld(Indexer indexer)
        {
            super(() -> {}, () -> {}, (Boolean b) -> {}, indexer::getIntakeBallLoaded, indexer);
        }
    }

    public class LoadBallToSlot extends CommandBase
    {
        private Indexer mIndexer;

        public LoadBallToSlot(Indexer indexer)
        {
            mIndexer = indexer;
            addRequirements(indexer);
        }

        public void initialize()
        {
        }

        public void execute()
        {
            if (mIndexer.getSlotBallLoaded() && !mIndexer.getIntakeBallLoaded()
                && mIndexer.isInSafeSpace())
            {
                mIndexer.transfer();
            }
            else
                mIndexer.endTransfer();
        }

        public void end(boolean interrupted)
        {
            mIndexer.endTransfer();
        }

        public boolean isFInished()
        {
            return (!mIndexer.getSlotBallLoaded() && mIndexer.getIntakeBallLoaded());
        }
    }

    public class ZeroSpinnerCommand extends CommandBase
    {

        private Indexer mIndexer;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ZeroSpinnerCommand(Indexer indexer)
        {
            mIndexer = indexer;
            addRequirements(mIndexer);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            mIndexer.spinAt(0.1);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            // ex. mClimber.raise();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished()
        {
            return mIndexer.checkFlag();
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            mIndexer.setZero();
            mIndexer.stopSpinner();
            mIndexer.returnToHome();
        }
    }

    /**
     * An {@link InstantCommand} runs an action and immediately exits.
     * <p>
     * @param Runnable toRun A reference to a subsystem method
     * @param Subsystem requirement For both the CommandScheduler and the above method reference.
     */

    /**
     * This {@link InstantCommand} stops the intake by calling
     * {@link Intake}.stop once.
     * <p>
     * Note that the Intake only controls the front roller.
     */
    public class StartLaunch extends InstantCommand
    {
        public StartLaunch(Indexer Indexer)
        {
            super(Indexer::launch, Indexer);
        }
    }

    public class EndLaunch extends InstantCommand
    {
        public EndLaunch(Indexer Indexer)
        {
            super(Indexer::endLaunch, Indexer);
        }
    }

    public class StartTransfer extends InstantCommand
    {
        public StartTransfer(Indexer Indexer)
        {
            super(Indexer::transfer, Indexer);
        }
    }

    public class EndTransfer extends InstantCommand
    {
        public EndTransfer(Indexer Indexer)
        {
            super(Indexer::endTransfer, Indexer);
        }
    }

    public class Spin extends InstantCommand
    {
        public Spin(Indexer Indexer, int N)
        {
            super(() -> Indexer.rotateN(N), Indexer);
        }
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
     * This {@link StartEndCommand} runs the intake motor backwards by calling
     * {@link Intake}.reverse repeatedly.
     * <p>
     * Note that this is not an Unjam command. The {@link Intake} subsystem only
     * controls the mechanical vector roller.
     */
    // public class WaitForBallHeld extends StartEndCommand
    // {
    // public WaitForBallHeld(Indexer indexer)
    // {
    // super(() -> {}, Indexer::getIntakeBallLoaded, indexer);
    // }
    // }

    public class Intake extends SequentialCommandGroup
    {
        private Indexer mIndexer;

        public Intake(Indexer indexer)
        {
            mIndexer = indexer;

            addCommands(new EndLaunch(indexer), // for safety
                new WaitForBallHeld(indexer), new LoadBallToSlot(indexer), new Spin(indexer, 1),
                new InstantCommand(() -> indexer.addBalls(1), indexer), new Intake(indexer) // recursions
            );
        }

        @Override
        public boolean isFinished()
        {
            return (mIndexer.getIntakeBallLoaded() && mIndexer.getSlotBallLoaded());
        }
    }

    public class Launch extends SequentialCommandGroup
    {
        private Indexer mIndexer;

        public Launch(Indexer indexer, int ballsToShoot)
        {
            mIndexer = indexer;

            addCommands(
                new StartLaunch(mIndexer),
                new ParallelCommandGroup(
                    new Spin(mIndexer, ballsToShoot),
                    new LoadBallToSlot(mIndexer)),
                new EndLaunch(mIndexer)
            );
        }
    }
}
