package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IndexerCommands
{
    private Indexer mIndexer;

    public IndexerCommands(Indexer indexer)
    {
        mIndexer = indexer;
        mIndexer.setDefaultCommand(new Stop());
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
            mIndexer.rotateN(mSpinCount);
        }

        @Override
        public void execute()
        {
            if (!mIndexer.getSlotBallLoaded() && mIndexer.areFinsAligned())
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
                mIndexer.unzero();
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

    public class Stop extends RunCommand
    {
        public Stop()
        {
            super(mIndexer::stop, mIndexer);
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
                new AlignIndexer(),
                new WaitForBallHeld(),
                new LoadBallToSlot(0),
                new StartTransfer(),
                    new WaitCommand(0.05),
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

    public class OptimizedLoadFromIntake extends SequentialCommandGroup
    {
        public OptimizedLoadFromIntake()
        {
            if (mIndexer.getSlotBallLoaded())
            {
                addCommands(
                    // new AlignIndexer(),
                    new WaitForBallHeld(),
                    new LoadBallToSlot(0),
                    new StartTransfer(),
                        new WaitCommand(0.3),
                    new EndTransfer(),
                    new SpinIndexer(1),
                    new InstantCommand(() -> mIndexer.addBalls(1), mIndexer)
                );
            }
            else
            {
                System.out.println("Leaving the ball in the outer slot... We have five balls held.");
                addCommands(
                    new WaitForBallHeld()
                );
            }
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
            return (mIndexer.getIntakeBallLoaded() && mIndexer.getSlotBallLoaded()) || super.isFinished();
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
            double spinDistance = 0.25; // (mIndexer.getSlotBallLoaded() && mIndexer.getIntakeBallLoaded()) ? 0.25 : 0;
            mIndexer.addBalls(-ballsToShoot);
            addCommands(
                // new AlignIndexer(mIndexer),
                ballsToShoot >= 5 ? new SpinIndexer(-spinDistance) : new SpinIndexer(spinDistance),
                new ParallelRaceGroup(
                    /**
                     * NOTE: Running a command without a subsystem is a Very Bad Practice,
                     * but here (in the RunCommand) it is the cleanest and safest way to
                     * deal with the hardware complexity of the whole launch system.
                     *
                     * We do this for a couple of reasons:
                     * 1. to adhere to motor safety by continuously calling launch()
                     * 2. because the alternative is a series of start and end commands
                     * 3. as the default command is stop(), the kicker can't accidentally stay on
                     */
                    new RunCommand(mIndexer::launch),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        ballsToShoot >= 5 ? new LoadBallToSlot(1 + spinDistance) : new InstantCommand(),
                        ballsToShoot >= 5 ? new ParallelCommandGroup(new WaitCommand(0.25), new StartTransfer()) : new InstantCommand(),
                        new WaitCommand(0.25),
                        new SpinIndexer(ballsToShoot >= 5 ? ballsToShoot - 1 : ballsToShoot - 0.25),
                        new EndTransfer()
                    )
                ),
                new InstantCommand(() -> mIndexer.addBalls(-ballsToShoot), mIndexer)
            );
        }
    }
}
