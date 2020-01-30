package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.PanelRotator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerPanelRotatorCommand extends CommandBase
{

    private PanelRotator mPanelRotator;

    // You should only use one subsystem per command. If multiple are needed, use a
    // CommandGroup.
    public LowerPanelRotatorCommand(PanelRotator mSubsystem)
    {
        mPanelRotator = mSubsystem;
        addRequirements(mPanelRotator);
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
        mPanelRotator.lower();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return mPanelRotator.getBeamSensorDown();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        mPanelRotator.stopExtendMotor();
    }
}
