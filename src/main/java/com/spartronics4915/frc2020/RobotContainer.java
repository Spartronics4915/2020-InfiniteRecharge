package com.spartronics4915.frc2020;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

import com.spartronics4915.frc2020.TrajectoryContainer.Destination;
import com.spartronics4915.frc2020.commands.*;
import com.spartronics4915.lib.hardware.sensors.T265Camera;
import com.spartronics4915.lib.hardware.sensors.T265Camera.CameraJNIException;
import com.spartronics4915.lib.math.twodim.control.RamseteTracker;
import com.spartronics4915.lib.subsystems.drive.CharacterizeDriveBaseCommand;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.subsystems.drive.TrajectoryTrackerCommand;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;
import com.spartronics4915.lib.util.Kinematics;
import com.spartronics4915.frc2020.subsystems.*;
import com.spartronics4915.frc2020.subsystems.LED.BlingState;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer
{
    private static class AutoMode
    {
        public final String name;
        public final Command command;

        public AutoMode(String name, Command command)
        {
            this.name = name;
            this.command = command;
        }
    }

    private static final String kAutoOptionsKey = "AutoStrategyOptions";
    public static final AutoMode kDefaultAutoMode = new AutoMode("All: Do Nothing", new Command()
    {
        @Override
        public Set<Subsystem> getRequirements()
        {
            return Set.of();
        }
    });

    public final NetworkTableEntry mAutoModeEntry = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard").getEntry("AutoStrategy");
    public final AutoMode[] mAutoModes;

    private final Climber mClimber;
    private final Intake mIntake;
    private final Launcher mLauncher;
    private final PanelRotator mPanelRotator;
    private final LED mLED;
    private final ClimberCommands mClimberCommands;
    private final LauncherCommands mLauncherCommands;
    private final PanelRotatorCommands mPanelRotatorCommands;
    private final ExampleCommandFactory mExampleCommandFactory;

    private final Joystick mJoystick;
    private final Joystick mButtonBoard;

    private final Drive mDrive;
    private final RamseteTracker mRamseteController = new RamseteTracker(2, 0.7);
    private final RobotStateEstimator mStateEstimator;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        mLauncher = new Launcher();
        mClimber = new Climber();
        mIntake = new Intake();
        mPanelRotator = new PanelRotator();
        mLED = LED.getInstance();
        mClimberCommands = new ClimberCommands();
        mLauncherCommands = new LauncherCommands();
        mPanelRotatorCommands = new PanelRotatorCommands();
        mExampleCommandFactory = new ExampleCommandFactory(mLED);

        //mClimber.setDefaultCommand(mClimberCommands.new ClimberDefaultCommand(mClimber));
        //mIntake.setDefaultCommand(mIntakeCommands.new IntakeDefaultCommand(mIntake));
        mLauncher.setDefaultCommand(mLauncherCommands.new LauncherDefaultCommand(mLauncher));
        //mPanelRotator.setDefaultCommand(mPanelRotatorCommands.new PanelRotatorDefaultCommand(mPanelRotator));

        mJoystick = new Joystick(Constants.OI.kJoystickId);
        mButtonBoard = new Joystick(Constants.OI.kButtonBoardId);

        T265Camera slamra;
        try
        {
            slamra = new T265Camera(Constants.Estimator.kCameraOffset,
                Constants.Estimator.kMeasurementCovariance);
        }
        catch (CameraJNIException e)
        {
            slamra = null;
        }
        mDrive = new Drive();
        mStateEstimator = new RobotStateEstimator(mDrive,
            new Kinematics(Constants.Drive.kTrackWidthMeters, Constants.Drive.kScrubFactor),
            slamra);

        configureJoystickBindings();
        configureButtonBoardBindings();

        System.out.println(TrajectoryContainer.middle.getTrajectory(null, Destination.ShieldGeneratorFarRight));

        mAutoModes = new AutoMode[] {kDefaultAutoMode,
            new AutoMode("Drive Straight",
                new TrajectoryTrackerCommand(mDrive,
                    TrajectoryContainer.middle.getTrajectory(Destination.ShieldGeneratorFarRight),
                    mRamseteController, mStateEstimator.getCameraRobotStateMap())),
            new AutoMode("Characterize Drive",
                new CharacterizeDriveBaseCommand(mDrive, Constants.Drive.kWheelDiameter))};

        String autoModeList = Arrays.stream(mAutoModes).map((m) -> m.name)
            .collect(Collectors.joining(","));
        SmartDashboard.putString(kAutoOptionsKey, autoModeList);
    }

    private void configureJoystickBindings()
    {
        // Note: changes to bling state can be augmented with:
        // .alongWith(new SetBlingStateCommand(mLED, BlingState.SOME_STATE)));

        /*
        new JoystickButton(mJoystick, 1).whenPressed(() -> mDrive.driveSlow()).whenReleased(() -> mDrive.driveNormal());
        new JoystickButton(mJoystick, 2).whenHeld(new LauncherCommands.Raise(mLauncher));
        new JoystickButton(mJoystick, 3).whenHeld(new LauncherCommands.Lower(mLauncher));
        new JoystickButton(mJoystick, 4).whenHeld(new LauncherCommands.Left(mLauncher));
        new JoystickButton(mJoystick, 5).whenHeld(new LauncherCommands.Right(mLauncher));
        */

        /* Switch Camera views
        new JoystickButton(mJoystick, 6).whenPressed(
                new InstantCommand(() -> mCamera.switch(Constants.Camera.kFrontId)));
        new JoystickButton(mJoystick, 7).whenPressed(
                new InstantCommand(() -> mCamera.switch(Constants.Camera.kRearId)));
        new JoystickButton(mJoystick, 10).whenPressed(
                new InstantCommand(() -> mCamera.switch(Constants.Camera.kIntakeId)));
        new JoystickButton(mJoystick, 11).whenPressed(
                new InstantCommand(() -> mCamera.switch(Constants.Camera.kTurretId)));
        */
        //new JoystickButton(mJoystick, 1).toggleWhenPressed(mLauncherCommands.new ShootBallTest(mLauncher));
        new JoystickButton(mJoystick, 2).toggleWhenPressed(mLauncherCommands.new TurretTest(mLauncher));
        new JoystickButton(mJoystick, 3).toggleWhenPressed(mLauncherCommands.new HoodTest(mLauncher));
        //new JoystickButton(mJoystick, 7).whileHeld(new TrajectoryTrackerCommand(mDrive,
        //    throughTrench(), mRamseteController, mStateEstimator.getCameraRobotStateMap()));
    }

    private void configureButtonBoardBindings()
    {
        /*
        new JoystickButton(mButtonBoard, 0).whenPressed(new IntakeCommands.Intake(mIntake));
        new JoystickButton(mButtonBoard, 1).whenPressed(new IntakeCommands.Stop(mIntake));
        new JoystickButton(mButtonBoard, 2).whileHeld(new IntakeCommands.Unjam(mIntake));
        */

        /*
        new JoystickButton(mButtonBoard, 3).whenPressed(new LauncherCommands.AimLow(mLauncher));
        new JoystickButton(mButtonBoard, 4).whenPressed(new LauncherCommands.Launch(mLauncher));
        new JoystickButton(mButtonBoard, 5).whenPressed(new LauncherCommands.AimHigh(mLauncher));
        */

        new JoystickButton(mButtonBoard, 6).whenPressed(mPanelRotatorCommands.new Raise(mPanelRotator));
        new JoystickButton(mButtonBoard, 7).whenPressed(mPanelRotatorCommands.new Lower(mPanelRotator));
        new JoystickButton(mButtonBoard, 8)
            .whenPressed(mPanelRotatorCommands.new SpinToColor(mPanelRotator));
        new JoystickButton(mButtonBoard, 9)
            .whenPressed(mPanelRotatorCommands.new SpinRotation(mPanelRotator));

        new JoystickButton(mButtonBoard, 10).whileHeld(mClimberCommands.new Extend(mClimber));
        new JoystickButton(mButtonBoard, 11).whileHeld(mClimberCommands.new Retract(mClimber));
        new JoystickButton(mButtonBoard, 14).whenHeld(mClimberCommands.new WinchPrimary(mClimber)
            .andThen(mClimberCommands.new WinchSecondary(mClimber)));

        /*
        new JoystickButton(mButtonBoard, 15).whenHeld(new TurretRaiseCommand(mLauncher));
        new JoystickButton(mButtonBoard, 16).whenHeld(new TurretLowerCommand(mLauncher));
        new JoystickButton(mButtonBoard, 17).whenHeld(new TurretLeftCommand(mLauncher));
        new JoystickButton(mButtonBoard, 18).whenHeld(new TurretRightCommand(mLauncher));
        */
    }

    // configureTestCommands is not actually run. It is declared public to
    // quell warnings. Its use is to test out different construction idioms 
    // for externally defined commands. 
    public void configureTestCommands()
    {
        // in this style object construction happens in the CommandFactory
        this.mExampleCommandFactory.MakeCmd(ExampleCommandFactory.CmdEnum.kTest1);

        // in this mode we construct things here, we must pass in parameters
        // that are required during construction, since the outer class 
        // member variables aren't accessible until after construction.
        this.mExampleCommandFactory.new Test5(this.mLED); // an InstantCommand
        this.mExampleCommandFactory.new Test6(this.mLED); // a StartEndCommand
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        String selectedModeName = mAutoModeEntry.getString("NO SELECTED MODE!!!!");
        Logger.notice("Auto mode name " + selectedModeName);
        for (var mode : mAutoModes)
        {
            if (mode.name.equals(selectedModeName))
            {
                return mode.command;
            }
        }

        Logger.error("AutoModeSelector failed to select auto mode: " + selectedModeName);
        return kDefaultAutoMode.command;
    }

    public TimedTrajectory<Pose2dWithCurvature> throughTrench()
    {
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d[] intermediate = new Pose2d[] {
            new Pose2d(Units.inchesToMeters(424), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(207), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180))};
        for (int i = 0; i < intermediate.length; i++)
        {
            Pose2d pose = intermediate[i];
            intermediate[i] = new Pose2d(pose.getTranslation().getX() - Units.inchesToMeters(312.5),
                pose.getTranslation().getY(), pose.getRotation());
        }
        RobotStateMap stateMap = mStateEstimator.getCameraRobotStateMap();
        Pose2d robotPose = stateMap.getLatestState().pose;
        double robotX = robotPose.getTranslation().getX();
        if (robotX < Units.inchesToMeters(312.5))
        {
            for (int i = 0; i < intermediate.length; i++)
            {
                Pose2d pose = intermediate[i];
                intermediate[i] = new Pose2d(-pose.getTranslation().getX(),
                    pose.getTranslation().getY(), Rotation2d.fromDegrees(0));
            }
        }
        for (int i = 0; i < intermediate.length; i++)
        {
            Pose2d pose = intermediate[i];
            intermediate[i] = new Pose2d(pose.getTranslation().getX() + Units.inchesToMeters(312.5),
                pose.getTranslation().getY(), pose.getRotation());
        }
        waypoints.add(robotPose);
        for (Pose2d pose : intermediate)
        {
            waypoints.add(pose);
        }
        ArrayList<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<TimingConstraint<Pose2dWithCurvature>>();
        return TrajectoryContainer.generateTrajectory(waypoints, constraints);
    }
}
