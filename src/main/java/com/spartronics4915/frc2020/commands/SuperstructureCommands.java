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
    public IndexerCommands mIndexerCommands;
    public IntakeCommands mIntakeCommands;
    public LauncherCommands mLauncherCommands;

    public SuperstructureCommands(RobotStateMap stateMap, Pose2d targetPose)
    {
        mIndexerCommands = new IndexerCommands();
        mIntakeCommands = new IntakeCommands();
        mLauncherCommands = new LauncherCommands(stateMap, targetPose);
    }

    public class LaunchSequence extends SequentialCommandGroup
    {
        public LaunchSequence(Indexer indexer, Launcher launcher)
        {
            addCommands(
                mLauncherCommands.new WaitForFlywheel(launcher),
                mIndexerCommands.new LoadToLauncher(indexer)
            );
        }
    }

    public class IntakeRace extends ParallelRaceGroup
    {
        public IntakeRace(Indexer indexer, Intake intake)
        {
            addCommands(
                mIntakeCommands.new Harvest(intake),
                mIndexerCommands.new LoadFromIntake(indexer)
            );
        }
    }

}