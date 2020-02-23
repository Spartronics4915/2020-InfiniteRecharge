package com.spartronics4915.frc2020.commands;

import java.util.Set;

import com.spartronics4915.frc2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IndexerCommands
{
    private Indexer mIndexer;

    public IndexerCommands(Indexer indexer)
    {
        mIndexer = indexer;
        // TODO: setDefaultCommand
        // mIndexer.setDefaultCommand(mIndexerCommands.new ZeroAndStopGroup(mIndexer));
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
        public WaitForBallHeld()
        {
            super(mIndexer::getIntakeBallLoaded);
        }

        @Override
        public void initialize()
        {
            super.initialize();
            System.out.println("WaitForBallHeld");
        }
    }

    /**
     * This {@link SequentialCommandGroup} aligns the spindexer, then rotates it.
     * @param spinCount a number of quarter rotations to spin
     */
    public class LoadBallToSlotGroup extends SequentialCommandGroup
    {
        public LoadBallToSlotGroup(int spinCount)
        {
            addCommands(
                new AlignIndexer(),
                new LoadBallToSlot(spinCount)
            );
        }
    }

    /**
     * Loads the ball into the indexer by moving the spinny bit
     */
    private class LoadBallToSlot extends CommandBase
    {
        private double mSpinCount;

        public LoadBallToSlot(double spinCount)
        {
            mSpinCount = spinCount;
            addRequirements(mIndexer);
        }

        @Override
        public void initialize()
        {
            System.out.println("LoadBallToSlot");
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
        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ZeroSpinnerCommand(boolean unzero)
        {
            if (unzero)
            {
                mIndexer.unzero();
            }

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
            return mIndexer.checkFlag() || mIndexer.hasZeroed();
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

    public class Stop extends CommandBase
    {
        @Override
        public void execute()
        {
            mIndexer.stop();
        }

        @Override
        public boolean isFinished()
        {
            return false;
        }

        @Override
        public Set<Subsystem> getRequirements()
        {
            return Set.of(mIndexer);
        }
    }

    /**
     * Starts the "popper" motor sending the ball into the launcher.
     * <p>
     * A workaround for being unable to schedule parallel commands within the same subsystem.
     */
    public class StartKicker extends InstantCommand
    {
        public StartKicker()
        {
            super(mIndexer::launch, mIndexer);
        }
    }

    /**
     * Stops the "popper" motor sending the ball into the launcher.
     * <p>
     * A workaround for being unable to schedule parallel commands within the same subsystem.
     */
    public class EndKicker extends InstantCommand
    {
        public EndKicker()
        {
            super(mIndexer::endLaunch, mIndexer);
        }

        @Override
        public void initialize()
        {
            super.initialize();
            System.out.println("EndKicker");
        }
    }

    /**
     * Starts the "loader" motor transferring the power cell into the indexer.
     * <p>
     * A workaround for being unable to schedule parallel commands within the same subsystem.
     */
    public class StartTransfer extends InstantCommand
    {
        public StartTransfer()
        {
            super(mIndexer::transfer, mIndexer);
        }
    }

    /**
     * Stops the "loader" motor transferring the power cell into the indexer.
     * <p>
     * A workaround for being unable to schedule parallel commands within the same subsystem.
     */
    public class EndTransfer extends InstantCommand
    {
        public EndTransfer()
        {
            super(mIndexer::endTransfer, mIndexer);
        }
    }

    /**
     * Spins the spindexer an arbitrary number of quarter-rotations.
     * @param n the number of quarter-rotations to spin (accepts doubles and negatives)
     */
    public class SpinIndexer extends CommandBase
    {
        private double quarterRotations;

        public SpinIndexer(double n)
        {
            quarterRotations = n;
            addRequirements(mIndexer);
        }

        @Override
        public void initialize()
        {
            mIndexer.rotateN(quarterRotations);
        }

        @Override
        public void execute()
        {
            mIndexer.goToPosition();
        }

        @Override
        public boolean isFinished()
        {
            return mIndexer.isAtPosition();
        }

        @Override
        public void end(boolean interrupted)
        {
            mIndexer.stopSpinner();
        }
    }

    /**
     * Moves the spindexer to the nearest quarter rotation.
     * <p>
     * Through use of Math.ceil, seems to only move clockwise.
     */
    public class AlignIndexer extends CommandBase
    {
        public AlignIndexer()
        {
            addRequirements(mIndexer);
        }

        @Override
        public void initialize()
        {
            mIndexer.toNearestQuarterRotation();
        }

        @Override
        public void execute()
        {
            mIndexer.goToPosition();
        }

        @Override
        public boolean isFinished()
        {
            return mIndexer.isAtPosition();
        }

        @Override
        public void end(boolean interrupted)
        {
            mIndexer.stopSpinner();
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
        public LoadFromIntake()
        {
            addCommands(
                new EndKicker(), // for safety
                // new AlignIndexer(),
                new WaitForBallHeld(),
                new LoadBallToSlot(0),
                new ParallelCommandGroup(
                    new WaitCommand(0.4), new StartTransfer()),
                new SpinIndexer(1),
                new EndTransfer(),
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
        public BulkHarvest()
        {
            for (int i = 0; i < 5; i++)
                addCommands(new LoadFromIntake());
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
        public LoadToLauncher(int ballsToShoot)
        {
            double spinDistance = (mIndexer.getSlotBallLoaded() && mIndexer.getIntakeBallLoaded()) ? 0.5 : 0;

            addCommands(
                // new AlignIndexer(mIndexer),
                new SpinIndexer(-spinDistance),
                new StartKicker(),
                new LoadBallToSlot(1 + spinDistance),
                new ParallelCommandGroup(
                    new WaitCommand(0.4), new StartTransfer()),
                new SpinIndexer(ballsToShoot - 1),
                new EndKicker(),
                new EndTransfer(),
                new InstantCommand(() -> mIndexer.addBalls(-ballsToShoot), mIndexer)
            );
        }

        public LoadToLauncher()
        {
            this(1);
        }
    }
}
