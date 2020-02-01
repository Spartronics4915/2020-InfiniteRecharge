package com.spartronics4915.frc2020;

import java.util.ArrayList;
import java.util.Set;

import com.spartronics4915.frc2020.TrajectoryContainer.Destination;
import com.spartronics4915.frc2020.commands.*;
import com.spartronics4915.lib.hardware.sensors.T265Camera;
import com.spartronics4915.lib.math.twodim.control.RamseteTracker;
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
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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

    public static final String kAutoOptionsKey = "AutoStrategyOptions";
    public static final AutoMode kDefaultAutoMode = new AutoMode("All: Do Nothing", new Command()
    {
        @Override
        public Set<Subsystem> getRequirements()
        {
            return null;
        }
    });

    public final NetworkTableEntry mAutoModeEntry = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard").getEntry("AutoModeStrategy");
    public final AutoMode[] mAutoModes;

    private Climber mClimber;
    private ClimberCommands mClimberCommands;

    private LED mLED;

    private Joystick mJoystick = new Joystick(Constants.OI.kJoystickId);
    private Joystick mButtonBoard = new Joystick(Constants.OI.kButtonBoardId);

    private final Drive mDrive;
    private final RamseteTracker mRamseteController = new RamseteTracker(2, 0.7);
    private final RobotStateEstimator mStateEstimator;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        mClimber = new Climber();
        mClimberCommands = new ClimberCommands(mClimber);
        mLED = LED.getInstance();

        configureJoystickBindings();
        configureButtonBoardBindings();

        mDrive = new Drive();
        mStateEstimator = new RobotStateEstimator(mDrive,
                new Kinematics(Constants.Drive.kTrackWidthMeters, Constants.Drive.kScrubFactor),
                new T265Camera(Constants.Estimator.kCameraOffset,
                        Constants.Estimator.kMeasurementCovariance));
        mAutoModes = new AutoMode[] {kDefaultAutoMode, new AutoMode("drive straight",
                new TrajectoryTrackerCommand(mDrive,
                        TrajectoryContainer.left.getTrajectory(null, Destination.LeftTrenchFar),
                        mRamseteController, mStateEstimator.getCameraRobotStateMap())),
                new AutoMode("drive through trench",
                        new TrajectoryTrackerCommand(mDrive, throughTrench(), mRamseteController,
                                mStateEstimator.getCameraRobotStateMap()))};

    }

    private void configureJoystickBindings()
    {
        // Note: changes to bling state can be augmented with:
        // .alongWith(new SetBlingStateCommand(mLED, BlingState.SOME_STATE)));

        /*
        new JoystickButton(mJoystick, 1).whileHeld(); // Slow the robot
        new JoystickButton(mJoystick, 2).whenHeld(new TurretRaiseCommand());
        new JoystickButton(mJoystick, 3).whenHeld(new TurretLowerCommand());
        new JoystickButton(mJoystick, 4).whenHeld(new TurretLeftCommand());
        new JoystickButton(mJoystick, 5).whenHeld(new TurretRightCommand());
        new JoystickButton(mJoystick, 6).whenPressed(); // Switch Camera views
        new JoystickButton(mJoystick, 7).whenPressed();
        new JoystickButton(mJoystick, 10).whenPressed();
        new JoystickButton(mJoystick, 11).whenPressed();
        */
        new JoystickButton(mJoystick, 1).toggleWhenPressed(new ShootBallTest(new Launcher()));
        new JoystickButton(mJoystick, 7).whileHeld(new TrajectoryTrackerCommand(mDrive,
                throughTrench(), mRamseteController, mStateEstimator.getCameraRobotStateMap()));
    }

    private void configureButtonBoardBindings()
    {
        /*
        new JoystickButton(mButtonBoard, 0).whenPressed(new IntakeCommand());
        new JoystickButton(mButtonBoard, 1).cancelWhenPressed(new IntakeCommand());
        new JoystickButton(mButtonBoard, 2).whileHeld(new UnjamCommand()); // Will reschedule itself
        new JoystickButton(mButtonBoard, 3).whenPressed(new AimLowCommand());
        new JoystickButton(mButtonBoard, 4).whenPressed(new LaunchCommand());
        new JoystickButton(mButtonBoard, 5).whenPressed(new AimHighCommand());
        new JoystickButton(mButtonBoard, 6).whenPressed(new PanelRaiseCommand());
        new JoystickButton(mButtonBoard, 7).whenPressed(new PanelLowerCommand());
        new JoystickButton(mButtonBoard, 8).whenPressed(new PanelSpinColorCommand());
        new JoystickButton(mButtonBoard, 9).whenPressed(new PanelSpinRotationsCommand());
        new JoystickButton(mButtonBoard, 10).whileHeld(new ClimberExtendCommand());
        new JoystickButton(mButtonBoard, 11).whileHeld(new ClimberRetractCommand());
        new JoystickButton(mButtonBoard, 14).whenHeld(new ClimberWinchCommand()); // Consists of a CommandGroup - ClimberWinchPrimary and ClimberWinchSecondary
        new JoystickButton(mButtonBoard, 15).whenHeld(new TurretRaiseCommand()); // Will not reschedule itself eg. voltage drop
        new JoystickButton(mButtonBoard, 16).whenHeld(new TurretLowerCommand());
        new JoystickButton(mButtonBoard, 17).whenHeld(new TurretLeftCommand());
        new JoystickButton(mButtonBoard, 18).whenHeld(new TurretRightCommand());
        */
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
        Pose2d[] intermediate = new Pose2d[]
        {new Pose2d(Units.inchesToMeters(424), Units.inchesToMeters(135),
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
