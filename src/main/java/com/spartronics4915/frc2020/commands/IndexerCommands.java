package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Indexer;
import com.spartronics4915.frc2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
    
    public class RunStateMachine extends CommandBase
    {
        private Indexer mIndexer;

        private boolean mLoad = false;

        public RunStateMachine(Indexer indexer)
        {
            mIndexer = indexer;
            addRequirements(indexer);
        }

        public void initialize() {
            if (mIndexer.getSlotBallLoaded() && !mIndexer.getIntakeBallLoaded()) {
                mLoad = true;
                return;
            }
            mLoad = false;
        }

        public void execute() {
            if (mLoad) {
                mIndexer.transfer();
            }
        }

        public void end(boolean interrupted) {
            mIndexer.endTransfer();
        }

        public boolean isFInished() {
            return (!mIndexer.getSlotBallLoaded() && mIndexer.getIntakeBallLoaded());
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
    public class Launch extends InstantCommand
    {
        public Launch(Indexer Indexer)
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

    public class Transfer extends InstantCommand
    {
        public Transfer(Indexer Indexer)
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
    //     public WaitForBallHeld(Indexer indexer)
    //     {
    //         super(() -> {}, Indexer::getIntakeBallLoaded, indexer);
    //     }
    // }

    public class Intake extends SequentialCommandGroup
    {
        private Indexer mIndexer;

        public Intake(Indexer indexer)
        {
            mIndexer = indexer;

            addCommands(
                new WaitForBallHeld(indexer),
                new RunStateMachine(indexer),
                new Spin(indexer, 1)
            );            
        }

        @Override
        public boolean isFinished()
        {
            return mIndexer.getIntakeBallLoaded() && mIndexer.getSlotBallLoaded();
        }
    }
}