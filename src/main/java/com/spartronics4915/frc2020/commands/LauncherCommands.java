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
            // TODO: Pull from vision + RobotStateMap + ILT
            mLauncher.runFlywheel(0);
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
}
