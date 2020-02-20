package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Indexer;
import com.spartronics4915.frc2020.subsystems.Intake;
import com.spartronics4915.frc2020.subsystems.Launcher;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperstructureCommands
{
    private IndexerCommands mIndexerCommands;
    private IntakeCommands mIntakeCommands;
    private LauncherCommands mLauncherCommands;

    private Indexer mIndexer;
    private Intake mIntake;
    private Launcher mLauncher;

    public SuperstructureCommands(LauncherCommands launcherCommands,
                                  IntakeCommands intakeCommands,
                                  IndexerCommands indexCommands)
    {
        mLauncherCommands = launcherCommands;
        mIntakeCommands = intakeCommands;
        mIndexerCommands = indexCommands;

        mLauncher = mLauncherCommands.getLauncher();
        mIntake = mIntakeCommands.getIntake();
        mIndexer = mIndexerCommands.getIndexer();
    }

    public class LaunchSequence extends SequentialCommandGroup
    {
        public LaunchSequence(int ballsToShoot)
        {
            addCommands(
                mLauncherCommands.new WaitForFlywheel(mLauncher),
                mIndexerCommands.new LoadToLauncher(mIndexer, ballsToShoot)
            );
        }
    }

    public class IntakeRace extends ParallelRaceGroup
    {
        public IntakeRace()
        {
            addCommands(
                mIntakeCommands.new Harvest(mIntake),
                mIndexerCommands.new LoadFromIntake(mIndexer)
            );
        }
    }

}