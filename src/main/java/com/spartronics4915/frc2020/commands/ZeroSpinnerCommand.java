package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroSpinnerCommand extends CommandBase
{

    private Indexer mIndexer;

    // You should only use one subsystem per command. If multiple are needed, use a
    // CommandGroup.
    public ZeroSpinnerCommand(Indexer indexer)
    {
        mIndexer = indexer;
        addRequirements(mIndexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        mIndexer.spinAt(0.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // ex. mClimber.raise();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return mIndexer.checkFlag();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        mIndexer.setZero();
        mIndexer.stopSpinner();
        mIndexer.returnToHome();
    }
}
