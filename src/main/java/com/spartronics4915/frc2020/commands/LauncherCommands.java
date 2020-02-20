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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LauncherCommands
{
    private final Launcher mLauncher;
    private final IndexerCommands mIndexerCommands;
    private final RobotStateMap mStateMap;
    private final Pose2d mTarget;

    public LauncherCommands(Launcher launcher, IndexerCommands indexerCommands, RobotStateMap stateMap)
    {
        mLauncher = launcher;
        mIndexerCommands = indexerCommands;
        mStateMap = stateMap;
        mTarget = null;
        // mLauncher.setDefaultCommand(new ConditionalCommand(mLauncherCommands.new TargetAndShoot(mLauncher),
        //     mLauncherCommands.new TrackPassively(mLauncher), mLauncher::inRange));
        // mLauncherCommands.new TargetAndShoot(mLauncher));
        mLauncher.setDefaultCommand(new ShootBallTest());
    }

    public class TargetAndShoot extends CommandBase
    {
        private Pose2d mTarget;

        public TargetAndShoot(Pose2d target)
        {
            mTarget = target;
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            double distance = trackTarget(mTarget);
            mLauncher.runFlywheel(mLauncher.calcRPS(distance));
        }

        @Override
        public void end(boolean interrupted)
        {
            mLauncher.stopTurret();
        }
    }

    public class TrackPassively extends CommandBase
    {
        private Pose2d mTarget;

        public TrackPassively(Pose2d target)
        {
            mTarget = target;
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            trackTarget(mTarget);
        }

        @Override
        public void end(boolean interrupted) {
            if(interrupted)
            {
                mLauncher.stopTurret();    
            }
        }
    }

    public class Zero extends CommandBase
    {
        public Zero()
        {
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
    private double trackTarget(Pose2d target)
    {
        Pose2d fieldToTurret = mStateMap.getLatestFieldToVehicle()
            .transformBy(Constants.Launcher.kRobotToTurret);
        Pose2d turretToTarget = fieldToTurret.inFrameReferenceOf(target);
        Rotation2d fieldAnglePointingToTarget =
            new Rotation2d(turretToTarget.getTranslation().getX(),
            turretToTarget.getTranslation().getY(), true);
        Rotation2d turretAngle = fieldAnglePointingToTarget
            .rotateBy(fieldToTurret.getRotation().inverse());
        double distance = mTarget.distance(fieldToTurret);
        mLauncher.adjustHood(mLauncher.calcPitch(distance));
        mLauncher.turnTurret(turretAngle);
        return distance;
    }

    /*
     * Command for testing, runs flywheel at a given RPS
     * !DO NOT MAKE THE RPS MORE THAN 90!
     */
    public class ShootBallTest extends CommandBase
    {
        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ShootBallTest()
        {
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.runFlywheel((double) mLauncher.dashboardGetNumber("flywheelRPSSlider", 0));
            mLauncher.adjustHood(Rotation2d.fromDegrees(
                (double) mLauncher.dashboardGetNumber("hoodAngleSlider", 0)));
            mLauncher.turnTurret(Rotation2d.fromDegrees(
                (double) mLauncher.dashboardGetNumber("turretAngleSlider", 0)));
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            if(interrupted) 
            {
                mLauncher.stopTurret();
            }
            mLauncher.reset();
        }
    }

    // XXX: subclass CommandBase so we don't need the launcher
    public class WaitForFlywheel extends WaitUntilCommand
    {
        public WaitForFlywheel()
        {
            super(mLauncher::isFlywheelSpun);
        }
    }

    /*
     * Command for testing, runs flywheel at a given RPS
     * !DO NOT MAKE THE RPS MORE THAN 90!
     */
    public class ShootBallTestWithDistance extends CommandBase
    {
        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ShootBallTestWithDistance()
        {
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.turnTurret(Rotation2d.fromDegrees((double) mLauncher.dashboardGetNumber("turretAngleSlider", 0)));
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
            if (interrupted) 
            {
                mLauncher.stopTurret();
            }
            mLauncher.reset();
        }
    }

    public class ShootingTest extends ParallelCommandGroup
    {
        public ShootingTest()
        {
            addCommands(new ShootBallTest(),
                mIndexerCommands.new LoadToLauncher(4));
        }
    }

    public class ShootingCalculatedTest extends ParallelCommandGroup
    {
        public ShootingCalculatedTest()
        {
            addCommands(new ShootBallTestWithDistance(),
                mIndexerCommands.new LoadToLauncher(4));
        }
    }

    public class TurretTest extends CommandBase
    {
        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public TurretTest()
        {
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            double degrees = (double) mLauncher.dashboardGetNumber("turretAngleSlider", 0);
            mLauncher.turnTurret(Rotation2d.fromDegrees(degrees));
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
                mLauncher.stopTurret();
        }
    }

    public class HoodTest extends CommandBase
    {
        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public HoodTest()
        {
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.adjustHood(
            Rotation2d.fromDegrees((double) mLauncher.dashboardGetNumber("hoodAngleSlider", 0)));
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
