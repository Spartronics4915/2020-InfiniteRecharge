package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class LauncherCommands
{
    public class ShootBallTest extends CommandBase
    {
    
        Launcher mLauncher;
    
        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ShootBallTest(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(launcher);
        }
    
        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            mLauncher.setRPS(40);
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

    /* */
    public class LauncherDefaultCommand extends CommandBase
    {
    
        Launcher mLauncher;
    
        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public LauncherDefaultCommand(Launcher launcher)
        {
            mLauncher = launcher;
            addRequirements(launcher);
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