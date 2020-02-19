package com.spartronics4915.frc2020.commands;

import java.util.function.BooleanSupplier;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.commands.IndexerCommands.LoadToLauncher;
import com.spartronics4915.frc2020.subsystems.Indexer;
import com.spartronics4915.frc2020.subsystems.Launcher;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LauncherCommands
{
    private final RobotStateMap mStateMap;
    private final Pose2d mTarget;

    public LauncherCommands(RobotStateMap stateMap, Pose2d targetPose)
    {
        mStateMap = stateMap;
        mTarget = targetPose;
    }

    public class TargetAndShoot extends CommandBase
    {
        private final Launcher mLauncher;

        public TargetAndShoot(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            double distance = trackTarget(mLauncher);
            mLauncher.runFlywheel(mLauncher.calcRPS(distance));
        }

        @Override
        public void end(boolean interrupted) {
            mLauncher.stopTurret();
        }
    }

    public class TrackPassively extends CommandBase
    {
        private final Launcher mLauncher;

        public TrackPassively(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            trackTarget(mLauncher);
        }
    }

    public class Zero extends CommandBase
    {
        private final Launcher mLauncher;

        public Zero(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        @Override
        public void execute()
        {
            mLauncher.zeroTurret();
        }

        @Override
        public boolean isFinished()
        {
            return mLauncher.isZeroed();
        }
    }

    /**
     * @return Distance to the target in meters
     */
    private double trackTarget(Launcher launcher)
    {
        Pose2d fieldToTurret = mStateMap.getLatestFieldToVehicle()
            .transformBy(Constants.Launcher.kRobotToTurret);
        Pose2d turretToTarget = fieldToTurret.inFrameReferenceOf(mTarget);
        Rotation2d fieldAnglePointingToTarget = new Rotation2d(
            turretToTarget.getTranslation().getX(), turretToTarget.getTranslation().getY(), true);

        Rotation2d turretAngle = fieldAnglePointingToTarget.rotateBy(fieldToTurret.getRotation().inverse());
        double distance = mTarget.distance(fieldToTurret);

        launcher.adjustHood(launcher.calcPitch(distance));
        launcher.turnTurret(turretAngle);

        return distance;
    }

    /*
     * Command for testing, runs flywheel at a given RPS
     * !DO NOT MAKE THE RPS MORE THAN 90!
     */
    public class ShootBallTest extends CommandBase
    {
        private final Launcher mLauncher;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ShootBallTest(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.runFlywheel((double) mLauncher.dashboardGetNumber("flywheelRPSSlider", 0));
            mLauncher.adjustHood(
                Rotation2d.fromDegrees((double) mLauncher.dashboardGetNumber("hoodAngleSlidder", 0)));
            mLauncher.turnTurret(Rotation2d.fromDegrees((double) mLauncher.dashboardGetNumber("turretAngleSlider", 0)));
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            mLauncher.reset();
        }
    }

    public class WaitForFlywheel extends WaitUntilCommand
    {
        public WaitForFlywheel(Launcher launcher)
        {
            super(() -> launcher.isFlywheelSpun());
        }
    }

    /*
     * Command for testing, runs flywheel at a given RPS
     * !DO NOT MAKE THE RPS MORE THAN 90!
     */
    public class ShootBallTestWithDistance extends CommandBase
    {
        private final Launcher mLauncher;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ShootBallTestWithDistance(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            double dist = (double) mLauncher.dashboardGetNumber("targetDistance", 120);
            mLauncher.runFlywheel(mLauncher.calcRPS(dist));
            mLauncher.adjustHood(mLauncher.calcPitch(dist));
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished()
        {
            return false;
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            //mLauncher.reset();
        }
    }

    public class ShootingTest extends ParallelCommandGroup
    {
        private Launcher mLauncher;
        private Indexer mIndexer;

        public ShootingTest(Launcher launcher, Indexer indexer)
        {
            mLauncher = launcher;
            mIndexer = indexer;
            addCommands(new ShootBallTest(mLauncher),
                (new IndexerCommands()).new LoadToLauncher(mIndexer, 4));
        }

    }

    public class ShootingCalculatedTest extends ParallelCommandGroup
    {
        private Launcher mLauncher;
        private Indexer mIndexer;

        public ShootingCalculatedTest(Launcher launcher, Indexer indexer)
        {
            mLauncher = launcher;
            mIndexer = indexer;
            addCommands(new ShootBallTestWithDistance(mLauncher),
                (new IndexerCommands()).new LoadToLauncher(mIndexer, 4));
        }

    }

    public class TurretTest extends CommandBase
    {
        private final Launcher mLauncher;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public TurretTest(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.turnTurret(Rotation2d.fromDegrees((double) mLauncher.dashboardGetNumber("TurretAimAngle", 0)));
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished()
        {
            return false;
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            //mLauncher.reset();
        }
    }

    public class HoodTest extends CommandBase
    {
        private final Launcher mLauncher;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public HoodTest(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.adjustHood(
                Rotation2d.fromDegrees((double) mLauncher.dashboardGetNumber("HoodAngle", 0)));
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished()
        {
            return false;
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            mLauncher.reset();
        }
    }
}
