package com.spartronics4915.frc2020.commands;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class IndexerCommandFactory
{
    private final SpartronicsSubsystem mSubsys;

    public IndexerCommandFactory(SpartronicsSubsystem subsys)
    {
        this.mSubsys = subsys;
    }

    public CommandBase GetCommand(String nm)
    {
        CommandBase result = null;
        switch (nm)
        {
            case "test1":
                result = new InstantCommand(() ->
                {
                    mSubsys.logInfo("running instant command test1");
                });
                break;
            case "test2":
                result = new InstantCommand(() ->
                {
                    mSubsys.logInfo("running instant command test2");
                });
                break;
            default:
        }
        return result;
    }

    public class Test3 extends CommandBase
    {

        private int mCount = 0;

        public Test3()
        {
            addRequirements(mSubsys);
        }

        public void execute()
        {
            mSubsys.dashboardPutNumber("executionCount", this.mCount++);
        }

        public boolean isFinished()
        {
            return this.mCount > 1000;
        }

        public void done()
        {
            mSubsys.logInfo("Test1 command is done");
        }
    }

    public class Test4 extends SequentialCommandGroup
    {
        public Test4()
        {
            addRequirements(mSubsys);

            // add subcommands here
        }
    }

    // Example InstantCommand. Why not just place the labmda expression in the
    // caller? - to encapsulate the details of the expression in this file
    // and not the caller (ownership of files is clearer)
    public class Test5 extends InstantCommand
    {
        public Test5(SpartronicsSubsystem subsys)
        {
            // Can't access mSubsys here, so we require that it be
            // passed in...
            super(() -> subsys.logInfo("InstantCommand Test5"));
        }
    }

    // Another subclassing example. Here we subclass StartEndCommand
    // and take care to pass our start and end Runnables as parameters
    // to the superclass constructor.
    public class Test6 extends StartEndCommand
    {
        public Test6(SpartronicsSubsystem subsys)
        {
            super(() ->
            {
                subsys.logInfo("Start of StartEndCommand");
            }, () ->
            {
                subsys.logInfo("End of StartEndCommand");
            }, subsys);
        }
    }
}
