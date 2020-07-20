package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.CoordSysMgr2020;
import com.spartronics4915.frc2020.subsystems.Launcher;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LauncherCommands
{
    private final Launcher mLauncher;
    private final IndexerCommands mIndexerCommands;
    final CoordSysMgr2020 mCoordSysMgr;

    public LauncherCommands(Launcher launcher, IndexerCommands indexerCommands)
    {
        mLauncher = launcher;
        mIndexerCommands = indexerCommands;
        mCoordSysMgr = new CoordSysMgr2020();

        // mLauncher.setDefaultCommand(new TargetAndShoot());
        mLauncher.setDefaultCommand(new ShootBallTest());
    }

    public Launcher getLauncher()
    {
        return mLauncher;
    }

    public class SetAsideToClimb extends CommandBase
    {
        public SetAsideToClimb()
        {
            addRequirements(mLauncher);
        }

        @Override
        public void execute()
        {
            mLauncher.turnTurret(Rotation2d.fromDegrees(-45));
        }

        @Override
        public boolean isFinished()
        {
            return false;
        }
    }

    public class TargetAndShoot extends CommandBase
    {
        public TargetAndShoot()
        {
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        // Default isFinished (true) is okay since we assume we'll be
        // interrupted by button-release.
        @Override
        public void execute()
        {
            double distance = trackTarget();
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
        public TrackPassively()
        {
            addRequirements(mLauncher);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            trackTarget();
        }

        @Override
        public void end(boolean interrupted)
        {
            if (interrupted)
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
     * @return Distance to the target in inches
     */
    private double trackTarget()
    {
        double distance = 0;
        distance = Units.metersToInches(distance);
        mLauncher.adjustHood(mLauncher.calcPitch(distance));
        mLauncher.turnTurret(0);
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
            if (interrupted)
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
            mLauncher.turnTurret(Rotation2d
                .fromDegrees((double) mLauncher.dashboardGetNumber("turretAngleSlider", 0)));
            double dist = (double) mLauncher.dashboardGetNumber("targetDistanceSlider", 120);
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
                Rotation2d
                    .fromDegrees((double) mLauncher.dashboardGetNumber("hoodAngleSlider", 0)));
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

    public class TurretVision extends CommandBase
    {
        double yam;
        double tomato;

        public TurretVision()
        {
            yam = 0;
            tomato = 0;

            addRequirements(mLauncher);
        }

        @Override
        public void execute()
        {
            mLauncher.turnTurret(Rotation2d.fromDegrees(tomato));
        }

        @Override
        public boolean isFinished()
        {
            return false;
        }

        @Override
        public void end(boolean interrupted)
        {
            mLauncher.stopTurret();
        }
    }
}
