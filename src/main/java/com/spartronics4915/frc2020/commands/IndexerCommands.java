package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Indexer;

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
     * Waits until a ball is held, then ends.
     */
     public class WaitForBallHeld extends FunctionalCommand
    {
        public WaitForBallHeld(Indexer indexer)
        {
            super(() -> {}, () -> {}, (Boolean b) -> {}, indexer::getIntakeBallLoaded, indexer);
        }
    }

    /**
     * Loads the ball into the indexer by moving the spinny bit
     */
    public class LoadBallToSlot extends CommandBase
    {
        private Indexer mIndexer;
        private int mSpinCount;

        public LoadBallToSlot(Indexer indexer, int spinCount)
        {
            mIndexer = indexer;
            mSpinCount = spinCount;
            addRequirements(mIndexer);
        }

        public void initialize()
        {
            mIndexer.rotateN(mSpinCount);
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

    /**
     * Moves the indexer to a zero position,
     * which is one of four perfectly aligned positions
     */
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
     * {@link LoadFromIntake}.stop once.
     * <p>
     * Note that the Intake only controls the front roller.
     */
    public class StartLaunch extends InstantCommand
    {
        public StartLaunch(Indexer indexer)
        {
            super(indexer::launch, indexer);
        }
    }

    public class EndLaunch extends InstantCommand
    {
        public EndLaunch(Indexer indexer)
        {
            super(indexer::endLaunch, indexer);
        }
    }

    public class StartTransfer extends InstantCommand
    {
        public StartTransfer(Indexer indexer)
        {
            super(indexer::transfer, indexer);
        }
    }

    public class EndTransfer extends InstantCommand
    {
        public EndTransfer(Indexer indexer)
        {
            super(indexer::endTransfer, indexer);
        }
    }

    public class Spin extends InstantCommand
    {
        public Spin(Indexer indexer, int N)
        {
            super(() -> indexer.rotateN(N), indexer);
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
     * {@link LoadFromIntake}.reverse repeatedly.
     * <p>
     * Note that this is not an Unjam command. The {@link LoadFromIntake} subsystem only
     * controls the mechanical vector roller.
     */

    public class LoadFromIntake extends SequentialCommandGroup
    {
        private Indexer mIndexer;

        public LoadFromIntake(Indexer indexer)
        {
            mIndexer = indexer;

            addCommands(
                new EndLaunch(mIndexer), // for safety
                new WaitForBallHeld(mIndexer), 
                new LoadBallToSlot(mIndexer, 0), 
                new Spin(mIndexer, 1),
                new InstantCommand(() -> mIndexer.addBalls(1), mIndexer), 
                new LoadFromIntake(mIndexer) // recursions
            );
        }

        @Override
        public boolean isFinished()
        {
            return (mIndexer.getIntakeBallLoaded() && mIndexer.getSlotBallLoaded());
        }
    }

    public class LoadToLauncher extends SequentialCommandGroup
    {
        private Indexer mIndexer;

        public LoadToLauncher(Indexer indexer, int ballsToShoot)
        {
            mIndexer = indexer;

            addCommands(
                new StartLaunch(mIndexer),
                new LoadBallToSlot(mIndexer, ballsToShoot),
                new EndLaunch(mIndexer)
            );
        }
    }
}
