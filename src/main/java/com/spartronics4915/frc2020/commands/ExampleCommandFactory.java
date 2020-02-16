package com.spartronics4915.frc2020.commands;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * Here's an example Command "Factory".
 * This example represents a way to have a per-subsystem "command factory"
 * that will allow us to factor/separate/encapsulate commands for
 * a subsystem into a single file, rather than mooshed into robotContainer
 * or spread across multiple files. Because java allows only a *single* class
 * to be represented in a sourcecode file, we utilize java "nested classes" to
 * define multiple classes in a single file. More detail can be found
 * here: https://docs.oracle.com/javase/tutorial/java/javaOO/nested.html.
 * 
 * There are two kinds of inner classes supported by java:
 * 
 *    1. static public class Foo
 *      A static inner class instance can be constructed without access to an 
 *      instance of the outer class.  Commands generally require access to one 
 *      or more subystems so minimimally:
 *          CommandBase b = new Outerclass.InnerStaticClass(mMySubsystem);
 * 
 *    2. public class Bar
 *      A regular inner class must be constructed via the 'new' method
 *      associated with the outer class *instance*.  Now we can implicity
 *      grant the inner class access to instance variables of the outer.
 *      Sharing data amongst many Commands might be accomplished via this
 *      pattern.  Minimally:
 *         CommandBase b = mMySubsystem.new InnerClass();
 * 
 *      This approach suffers when your InnerClass wants to inherit behavior
 *      from certain wpilibj commands that are tailored for inline
 *      construction.  One example of this is InstantCommand as extended
 *      in Test5 below.  In order to construct an instance of InstantCommand
 *      we must pass a Runnable as well as one or more subsystems via super(). 
 *      The only problem is that we can't access OuterClass instance variables 
 *      until after InnerClass is constructed.  To work around this, we must 
 *      pass in references to the required subsystem as well as any other 
 *      parameters to express the construction, like so:
 *          CommandBase b = mMySubsystem.new InnerClass(mMySubsystem);
 *      This feels a little weird. In the case of InstantCommand there
 *      is a remedy: InstantCommand has an empty constructor and we can
 *      simply override the minimal behavior.  See Test5a, below.
 *      This is not possible with FunctionalCommand and you should consider
 *      these comments from FunctionalCommand:
 *        A command that allows the user to pass in functions for each of 
 *        the basic command methods through the constructor.  Useful for 
 *        inline definitions of complex commands - note, however, that if a
 *        command is beyond a certain complexity it is usually better practice 
 *        to write a proper class for it than to inline it.
 *
 * Above we discuss two innerclass solutions. There is also a third
 * approach available to us as exemplified in our MakeCmd method, below.
 * Pick the best for your application, though option #1 is likely preferred.
 * 
 * Executive summary:
 *
 * 1. Contextualized inner class 
 *  - access to outerclass member variables, only not as args to super.
 *  - requires unusual, but legal, java syntax for construction
 *  - if you wish to subclass simple base classes (ie: InstantCommand, 
 *    StartStopCommand, FunctionalCommand) either follow example Test5a
 *    or simply bail on the use of FunctionalCommand in favor of CommandBase.
 *  - odd-looking constructor syntax:  mCommandFactory.new Test5a();
 * 
 * 2. Independant inner class
 *  - no access to outerclass member variables
 *  - yes access to outerclass static variables
 *  - standard constructor syntax:  new ExampleCommandFactory.Test7(mCameraSys)
 * 
 * 3. Enumerated MakeCmd method
 *  - easy to read and use, but the dispatch is switch-based
 *
 * Usage:
 *   This examples operates on a single subsystem.  Note that some commands may
 *   require or operate on more than one subsystem.  This example could be 
 *   extended to accept an ArrayList<SpartronicsSubsystem> or 
 *   Set<SpartronicsSubsystem>. Or explicitly require, say, the DriveTrain 
 *   subsystem.
 *
 *   in RobotContainer():
 *     this.mCamera = new CameraSubsystem();
 *      
 *     // ExampleCommandFactory constructor accepts subsystem and
 *     // misc parameters to store in the instance.  These will be
 *     // be available to methods of inner classes unless the methods
 *     // are defined in the constructor or passed to the super during
 *     // construction. 
 *     this.mCamCmds = new ExampleCommandFactory(mCamera, ...parameters...); 
 * 
 *   See RobotContainer::configureTestCommands(), for instantiatioon examples.
 *
 */
public class ExampleCommandFactory
{
    public enum CmdEnum
    {
        kTest1,
        kTest2
    };

    private final SpartronicsSubsystem mSubsys;

    public ExampleCommandFactory(SpartronicsSubsystem subsys)
    {
        this.mSubsys = subsys;
    }

    public CommandBase MakeCmd(CmdEnum c)
    {
        CommandBase result = null;
        switch (c)
        {
            case kTest1:
                result = new InstantCommand(() ->
                {
                    mSubsys.logInfo("running instant command test1");
                });
                break;
            case kTest2:
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

    // Example InstantCommand. Question: why not just place the lambda 
    // expression in the caller? Answer: to encapsulate the details of the 
    // expression in this file and not the caller (ownership of files is clearer)
    public class Test5 extends InstantCommand
    {
        public Test5(SpartronicsSubsystem subsys)
        {
            // Can't access outer class member variables in a Runnable passed
            // to super since it may run it before we're fully constructed.
            // We require that the subsystem be passed in if this InstantCommand
            // 'requires' it. Error message:
            //   cannot refer to 'this' nor 'super' while explicitly invoking a 
            //   constructorJava(134217866)
            super(() -> { subsys.logInfo("Run Test5"); }, subsys);
        }
    }

    // A variant of Test5 that relies on InstanceCommand default constructor.
    // And in doing so no longer requires that the subsys be passed to
    // the constructor. Only downside is that we must call addRequirements
    // and override the initialize method.
    public class Test5a extends InstantCommand
    {
        public Test5a()
        {
            super();
            this.addRequirements(mSubsys); // <--- access to OuterClass instance
            // we can't set InstantCommand::m_toRun since it's private
            // this.m_toRun = this::myInstantMethod();
        }

        @Override
        public void initialize()
        {
            // this is where InstantCommand does its thing
            mSubsys.logInfo("Run Test5a");
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
                // look we can put comments in our code!
                // this is the Start command
                subsys.logInfo("Start of StartEndCommand");
            }, () ->
            {
                // look we can put comments in our code!
                // this is the end command.
                subsys.logInfo("End of StartEndCommand");
            }, subsys);
        }
    }

    /**
     * Example variant of Test6 that implements StartEnd without
     * subclassing. Note that we haven't expressed an isFinished()
     * but perhaps we should?
     */
    public class Test6a extends CommandBase
    {
        public Test6a()
        {
            this.addRequirements(mSubsys);
        }

        @Override
        public void initialize()
        {
            mSubsys.logInfo("Start of StartEndCommand");
        }

        @Override
        public void end(boolean interrupted)
        {
            mSubsys.logInfo("End of StartEndCommand");
        }
    }

    // A public static (inner) class allows us to reference
    // this class without contructing an instance of our outer
    // class.  ie: we can be instantiated via: 
    //      new ExampleCommandFactory.Test7(mySubsys)
    public static class Test7 extends StartEndCommand
    {
        public Test7(SpartronicsSubsystem subsys)
        {
            super(() ->
            {
                // look we can put comments in our code!
                // this is the Start command
                subsys.logInfo("Start of Test7/StartEndCommand");
            }, () ->
            {
                // look we can put comments in our code!
                // this is the end command.
                subsys.logInfo("End of Test7/StartEndCommand");
            }, subsys);
        }
    }
}
