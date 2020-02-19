package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IndexerCommands
{
    private Indexer mIndexer;
    public IndexerCommands(Indexer indexer)
    {
        mIndexer = indexer;
    }

    public Indexer getIndexer()
    {
        return mIndexer;
    }

    /**
     * Waits until a ball is held, then ends.
     */
    public class WaitForBallHeld extends WaitUntilCommand
    {
        public WaitForBallHeld(Indexer indexer)
        {
            super(indexer::getIntakeBallLoaded);
        }

        @Override
        public void initialize()
        {
            super.initialize();
            System.out.println("here3");
        }
    }

    /**
     * This {@link SequentialCommandGroup} aligns the spindexer, then rotates it.
     * @param spinCount a number of quarter rotations to spin
     */
    public class LoadBallToSlotGroup extends SequentialCommandGroup
    {
        public LoadBallToSlotGroup(Indexer indexer, int spinCount)
        {
            addCommands(
                new AlignIndexer(indexer),
                new LoadBallToSlot(indexer, spinCount)
            );
        }
    }

    /**
     * Loads the ball into the indexer by moving the spinny bit
     */
    private class LoadBallToSlot extends CommandBase
    {
        private Indexer mIndexer;
        private double mSpinCount;

        public LoadBallToSlot(Indexer indexer, double spinCount)
        {
            mIndexer = indexer;
            mSpinCount = spinCount;
            addRequirements(mIndexer);
        }

        @Override
        public void initialize()
        {
            System.out.println("here4");
            mIndexer.rotateN(mSpinCount);
        }

        @Override
        public void execute()
        {
            if (!mIndexer.getSlotBallLoaded() && mIndexer.getIntakeBallLoaded()
                && mIndexer.areFinsAligned())
                mIndexer.transfer();
            else
                mIndexer.endTransfer();
            mIndexer.goToPosition();
        }

        @Override
        public void end(boolean interrupted)
        {
            mIndexer.endTransfer();
        }

        @Override
        public boolean isFinished()
        {
            return mIndexer.getSlotBallLoaded() && !mIndexer.getIntakeBallLoaded();
        }
    }

    /**
     * Moves the indexer to a zero position, which is one of four perfectly aligned positions
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

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mIndexer.spinAt(0.1);
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
     * Starts the "popper" motor sending the ball into the launcher.
     * <p>
     * A workaround for being unable to schedule parallel commands within the same subsystem.
     */
    public class StartKicker extends InstantCommand
    {
        public StartKicker(Indexer indexer)
        {
            super(indexer::launch, indexer);
        }
    }

    /**
     * Stops the "popper" motor sending the ball into the launcher.
     * <p>
     * A workaround for being unable to schedule parallel commands within the same subsystem.
     */
    public class EndKicker extends InstantCommand
    {
        public EndKicker(Indexer indexer)
        {
            super(indexer::endLaunch, indexer);
        }

        @Override
        public void initialize()
        {
            super.initialize();
            System.out.println("here2");
        }
    }

    /**
     * Starts the "loader" motor transferring the power cell into the indexer.
     * <p>
     * A workaround for being unable to schedule parallel commands within the same subsystem.
     */
    public class StartTransfer extends InstantCommand
    {
        public StartTransfer(Indexer indexer)
        {
            super(indexer::transfer, indexer);
        }
    }

    /**
     * Stops the "loader" motor transferring the power cell into the indexer.
     * <p>
     * A workaround for being unable to schedule parallel commands within the same subsystem.
     */
    public class EndTransfer extends InstantCommand
    {
        public EndTransfer(Indexer indexer)
        {
            super(indexer::endTransfer, indexer);
        }
    }

    /**
     * Spins the spindexer an arbitrary number of quarter-rotations.
     * @param N the number of quarter-rotations to spin (accepts doubles and negatives)
     */
    public class SpinIndexer extends FunctionalCommand
    {
        public SpinIndexer(Indexer indexer, double N)
        {
            super(
                () -> indexer.rotateN(N),
                indexer::goToPosition,
                (b) -> indexer.stopSpinner(),
                () -> indexer.isAtPosition(),
                indexer
            );
        }

        @Override
        public void initialize()
        {
            super.initialize();
            System.out.println("here5");
        }
    }

    /**
     * Moves the spindexer to the nearest quarter rotation.
     * <p>
     * Through use of Math.ceil, seems to only move clockwise.
     */
    public class AlignIndexer extends FunctionalCommand
    {
        public AlignIndexer(Indexer indexer)
        {
            super( 
                indexer::toNearestQuarterRotation, 
                indexer::goToPosition,
                (Boolean b) -> indexer.stopSpinner(),
                () -> indexer.isAtPosition(),
                indexer
            );
        }

        @Override
        public void initialize()
        {
            super.initialize();
            System.out.println("here");
        }
    }

    /**
     * Loads a ball from the intake slot by aligning {@link AlignIndexer}, waiting for a ball {@link WaitForBallHeld},
     * loading that ball {@link LoadBallToSlot}, and rotating {@link SpinIndexer} to a new available slot.
     * <p>
     * Terminates when there is a ball in both the first slot of the indexer and the intake.
     */
    public class LoadFromIntake extends SequentialCommandGroup
    {
        private Indexer mIndexer;

        public LoadFromIntake(Indexer indexer)
        {
            mIndexer = indexer;
            addCommands(
                new EndKicker(mIndexer), // for safety
                // new AlignIndexer(mIndexer),
                new WaitForBallHeld(mIndexer),
                new LoadBallToSlot(mIndexer, 0),
                new ParallelCommandGroup(
                    new WaitCommand(0.4),
                    new StartTransfer(mIndexer)
                ),
                new SpinIndexer(mIndexer, 1),
                new EndTransfer(mIndexer),
                new InstantCommand(() -> mIndexer.addBalls(1), mIndexer)
            );
        }

        @Override
        public boolean isFinished()
        {
            return (mIndexer.getIntakeBallLoaded() && mIndexer.getSlotBallLoaded()) || super.isFinished();
        }
    }

    /**
     * Queues the {@link LoadFromIntake} command five times.
     * <p>
     * Terminates on the same condition (redundant?)
     */
    public class BulkHarvest extends SequentialCommandGroup
    {
        private Indexer mIndexer;

        public BulkHarvest(Indexer indexer)
        {
            mIndexer = indexer;
            for (int i = 0; i < 5; i++)
                addCommands(new LoadFromIntake(mIndexer));
        }

        @Override
        public boolean isFinished()
        {
            return (mIndexer.getIntakeBallLoaded() && mIndexer.getSlotBallLoaded());
        }
    }
    /**
     * Loads a ball from the indexer to the launcher.
     * <p>
     * Aligns, spins to make room for the flywheel, starts the kicker, spins the indexer to load all balls, and stops the kicker.
     */
    public class LoadToLauncher extends SequentialCommandGroup
    {
        private Indexer mIndexer;
        public LoadToLauncher(Indexer indexer, int ballsToShoot)
        {
            mIndexer = indexer;
            double spinDistance = (mIndexer.getSlotBallLoaded() && mIndexer.getIntakeBallLoaded()) ? 0.5 : 0;

            addCommands(
                // new AlignIndexer(mIndexer),
                new SpinIndexer(mIndexer, -spinDistance),
                new StartKicker(mIndexer),
                new LoadBallToSlot(mIndexer, 1 + spinDistance),
                new ParallelCommandGroup(
                    new WaitCommand(0.4),
                    new StartTransfer(mIndexer)
                ),
                new SpinIndexer(mIndexer, ballsToShoot - 1),
                new EndKicker(mIndexer),
                new EndTransfer(mIndexer),
                new InstantCommand(() -> mIndexer.addBalls(-ballsToShoot), mIndexer)
            );
        }

        public LoadToLauncher(Indexer indexer)
        {
            this(indexer, 5);
        }
    }
}
