package com.spartronics4915.frc2020.commands;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Here's an example Command "Factory" that operates on a single
//  subsystem.  Note that some commands may require or operate on
//  more than one subsystem.  This example could be extended to
//  accept an ArrayList<SpartronicsSubsystem> or Set<SpartronicsSubsystem>
//
// This example represents a way to have a per-subsystem factory
// that will allow us to factor/separate/encapsulate commands for
// a subsystem into a single file, rather than mooshed into robotContainer
// or spread across multiple files.
//
// usage:
//  in RobotContainer():
//     mCameraCmdFactory = new ExampleCommandFactory(mCamera);
//
//  in RobotContainer::configureJoystickButtons()
//     new JoystickButton(...).whenPressed(mCameraCmdFactory.GetCommand("test1")
// 
public class ExampleCommandFactory
{
    private final SpartronicsSubsystem mSubsys;

    ExampleCommandFactory(SpartronicsSubsystem subsys)
    {
        this.mSubsys = subsys;
    }

    public CommandBase GetCommand(String nm)
    {
        CommandBase result = null;
        switch (nm)
        {
        case "test1":
            result = new InstantCommand(() -> mSubsys.logInfo("running instant command test1"));
            break;
        case "test2":
            result = new InstantCommand(() -> mSubsys.logInfo("running instant command test2"));
            break;
        default:
        }
        return result;
    }

    public class Test1 extends CommandBase
    {

        private int mCount = 0;

        public Test1()
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

    public class Test2 extends CommandGroupBase
    {

        Test2()
        {
            addRequirements(mSubsys);
        }

        public void addCommands(Command... commands)
        {
        }

        public void execute()
        {
        }
    }
}
