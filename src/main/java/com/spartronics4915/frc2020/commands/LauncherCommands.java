package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.subsystems.Launcher;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class LauncherCommands
{
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
            mLauncher.setRPS(SmartDashboard.getNumber("Launcher/FlywheelRPS", 0));
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.setRPS(SmartDashboard.getNumber("Launcher/FlywheelRPS", 0));
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
            SmartDashboard.putNumber("Launcher/TurretDirection", mLauncher.getTurretDirection());
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
            mLauncher.setPitch(SmartDashboard.getNumber("Launcher/HoodAngle", 0));
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mLauncher.setPitch(SmartDashboard.getNumber("Launcher/HoodAngle", 0));
            mLauncher.rotateHood();
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

    public class AutoAimTurret extends CommandBase {
        private final Launcher mLauncher;
        private final RobotStateMap mStateMap;
        private final Pose2d mTargetPose;

        public AutoAimTurret(Launcher launcher, Pose2d targetPose, RobotStateMap stateMap) {
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
            Rotation2d fieldAnglePointingToTarget = new Rotation2d(turretToTarget.getTranslation().getX(), turretToTarget.getTranslation().getY(), true).inverse();
            Rotation2d turretAngle = fieldAnglePointingToTarget.rotateBy(fieldToTurret.getRotation());
            if(turretAngle.getDegrees() > 45.0) {
                turretAngle = Rotation2d.fromDegrees(45.0);
            } else if (turretAngle.getDegrees() < -45.0) {
                turretAngle = Rotation2d.fromDegrees(-45.0);
            }
            mLauncher.turnTurret(turretAngle);
            mLauncher.setPitch(mLauncher.calcPitch(turretToTarget.distance(mTargetPose)).getDegrees());
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
