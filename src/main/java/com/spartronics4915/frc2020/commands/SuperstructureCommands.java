package com.spartronics4915.frc2020.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperstructureCommands
{
    private final IndexerCommands mIndexerCommands;
    private final IntakeCommands mIntakeCommands;
    private final LauncherCommands mLauncherCommands;

    public SuperstructureCommands(IndexerCommands indexerCommands,
        IntakeCommands intakeCommands, LauncherCommands launcherCommands)
    {
        mLauncherCommands = launcherCommands;
        mIntakeCommands = intakeCommands;
        mIndexerCommands = indexerCommands;
    }

    public class LaunchSequence extends SequentialCommandGroup
    {
        public LaunchSequence(int ballsToShoot)
        {
            addCommands(
                mLauncherCommands.new WaitForFlywheel(),
                mIndexerCommands.new LoadToLauncher(ballsToShoot)
            );
        }

        public LaunchSequence() // XXX: should this shoot > 1?
        {
            this(1);
        }
    }

    public class Intake extends ParallelCommandGroup
    {
        public Intake()
        {
            addCommands(
                mIntakeCommands.new Harvest(true),
                mIndexerCommands.new LoadFromIntake()
            );
        }
    }

    public class IntakeNoStop extends ParallelRaceGroup
    {
        public IntakeNoStop()
        {
            addCommands(
                mIntakeCommands.new Harvest(true),
                mIndexerCommands.new OptimizedLoadFromIntake()
            );
        }
    }

    public class IntakeFive extends SequentialCommandGroup
    {
        public IntakeFive()
        {
            addCommands(
                new Intake(),
                new Intake(),
                new Intake(),
                new Intake(),
                new Intake()
            );
        }
    }
}
