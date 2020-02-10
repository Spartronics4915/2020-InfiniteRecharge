package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.subsystems.Launcher;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * Command overview:
 *
 *
 * Indexer controls:
 *
 * VisionAutoAlign // NOTE: To my knowledge, revving takes longer than auto aligning, making a VisionAutoAlignWithoutRevving Command useless.
 *
 */
public class LauncherCommands
{
    public class Target extends CommandBase
    {
        private final Launcher mLauncher;

        public Target(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        @Override
        public void initialize()
        {
        }

        @Override
        public void execute()
        {
            mLauncher.runFlywheel();
        }

        @Override
        public boolean isFinished()
        {
            // return true if at the correct angle and rotation
            // will kick out to defaultcommand and continue to run until out of range
            return false;
        }

        @Override
        public void end(boolean interrupted)
        {

        }
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

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            mLauncher.setRPS((double) mLauncher.dashboardGetNumber("FlywheelRPS", 0));
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.setRPS((double) mLauncher.dashboardGetNumber("FlywheelRPS", 0));
            mLauncher.runFlywheel();
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
            mLauncher.dashboardPutNumber("Launcher/TurretDirection", mLauncher.getCurrentRotation());
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

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            mLauncher.setPitch((double) mLauncher.dashboardGetNumber("Launcher/TurretAimAngle", 0));
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.setPitch((double) mLauncher.dashboardGetNumber("Launcher/TurretAimAngle", 0));
            mLauncher.adjustHood();
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

    public class HoodToFieldPosition extends CommandBase
    {
        private final Launcher mLauncher;
        private final RobotStateMap mStateMap;
        private final Pose2d mTargetPose;

        public HoodToFieldPosition(Launcher launcher, Pose2d targetPose, RobotStateMap stateMap)
        {
            mLauncher = launcher;
            mTargetPose = targetPose;
            mStateMap = stateMap;
            addRequirements(mLauncher);
        }

        @Override
        public void execute()
        {
            Pose2d fieldToTurret = mStateMap.getLatestFieldToVehicle()
                .transformBy(Constants.Launcher.kTurretOffset);
            Pose2d turretToTarget = fieldToTurret.inFrameReferenceOf(mTargetPose);
            Rotation2d fieldAnglePointingToTarget = new Rotation2d(
                turretToTarget.getTranslation().getX(), turretToTarget.getTranslation().getY(),
                true).inverse();
            Rotation2d turretAngle = fieldAnglePointingToTarget
                .rotateBy(fieldToTurret.getRotation());
        }
    }

    /*
     * Default command of the launcher subsystem, makes the flywheel's target rps 0
     */
    public class LauncherDefaultCommand extends CommandBase
    {
        private final Launcher mLauncher;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public LauncherDefaultCommand(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(mLauncher);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            mLauncher.setRPS(0);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.runFlywheel();
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
