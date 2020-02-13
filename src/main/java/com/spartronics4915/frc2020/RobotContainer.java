package com.spartronics4915.frc2020;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

import com.spartronics4915.frc2020.TrajectoryContainer.Destination;
import com.spartronics4915.frc2020.commands.*;
import com.spartronics4915.frc2020.subsystems.*;
import com.spartronics4915.frc2020.subsystems.LED.BlingState;
import com.spartronics4915.lib.hardware.sensors.T265Camera;
import com.spartronics4915.lib.hardware.sensors.T265Camera.CameraJNIException;
import com.spartronics4915.lib.math.twodim.control.RamseteTracker;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rectangle2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.subsystems.drive.CharacterizeDriveBaseCommand;
import com.spartronics4915.lib.subsystems.drive.TrajectoryTrackerCommand;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;
import com.spartronics4915.lib.util.Kinematics;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

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

    /* subsystems */
    private final Climber mClimber;
    private final Intake mIntake;
    private final Launcher mLauncher;
    private final PanelRotator mPanelRotator;
    private final LED mLED;
    private final Vision mVision;

    /* subsystem commands */
    private final ClimberCommands mClimberCommands;
    private final IntakeCommands mIntakeCommands;
    private final LauncherCommands mLauncherCommands;
    private final PanelRotatorCommands mPanelRotatorCommands;

    private Joystick mJoystick = null; // only created if port is > -1
    private Joystick mButtonBoard = null; // only created if port is > -1

    private final Drive mDrive;
    private final RamseteTracker mRamseteController = new RamseteTracker(2, 0.7);
    private final RobotStateEstimator mStateEstimator;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        if(Constants.OI.kJoystickPort >= 0)
            mJoystick = new Joystick(Constants.OI.kJoystickPort);
        else
            Logger.notice("RobotContainer skipping joystick");

        if(Constants.OI.kButtonBoardPort >= 0)
            mButtonBoard = new Joystick(Constants.OI.kButtonBoardPort);
        else
            Logger.notice("RobotContainer skipping buttonboard");

        T265Camera slamra;
        try
        {
            slamra = new T265Camera(Constants.Estimator.kSlamraToRobot,
                Constants.Estimator.kMeasurementCovariance);
        }
        catch (CameraJNIException | UnsatisfiedLinkError e)
        {
            slamra = null;
            Logger.exception(e);
        }
        mDrive = new Drive();
        mStateEstimator = new RobotStateEstimator(mDrive,
            new Kinematics(Constants.Drive.kTrackWidthMeters, Constants.Drive.kScrubFactor),
            slamra);
        var slamraCommand = new StartEndCommand(() -> mStateEstimator.enable(),
            () -> mStateEstimator.stop(), mStateEstimator);
        mStateEstimator.setDefaultCommand(slamraCommand);

        // XXX: do we need to hold a reference the the DestinationCouple here?
        Logger.info("Hash code for initial trajectory:"  + 
            new TrajectoryContainer.DestinationCouple(Destination.ShieldGeneratorFarRight,
                Destination.MiddleShootingPosition).hashCode());

        mAutoModes = new AutoMode[]
        {
            kDefaultAutoMode,
            new AutoMode("Drive Straight",
                new TrajectoryTrackerCommand(mDrive,
                TrajectoryContainer.middle.getTrajectory(null, Destination.ShieldGeneratorFarRight),
                mRamseteController, mStateEstimator.getEncoderRobotStateMap())),
            new AutoMode("Characterize Drive",
                new CharacterizeDriveBaseCommand(mDrive, Constants.Drive.kWheelDiameter)),
            new AutoMode("Laser Turret",
                new LaserTurretToFieldPose(mStateEstimator.getCameraRobotStateMap())),
            new AutoMode("Right: Through Trench",
                new TrajectoryTrackerCommand(mDrive,
                    TrajectoryContainer.left.getTrajectory(null, Destination.LeftTrenchFar),
                    mRamseteController, mStateEstimator.getEncoderRobotStateMap()))
        };

        mStateEstimator.resetRobotStateMaps(TrajectoryContainer.middle.mStartPoint);

        String autoModeList = Arrays.stream(mAutoModes).map((m) -> m.name).collect(Collectors.joining(","));
        SmartDashboard.putString(kAutoOptionsKey, autoModeList);

        mClimber = new Climber();
        mIntake = new Intake();
        mLauncher = new Launcher();
        mPanelRotator = new PanelRotator();
        mLED = LED.getInstance();
        mVision = new Vision(mStateEstimator, mLauncher);

        mClimberCommands = new ClimberCommands();
        mIntakeCommands = new IntakeCommands();
        mLauncherCommands = new LauncherCommands(mStateEstimator.getCameraRobotStateMap(), new Pose2d());
        mPanelRotatorCommands = new PanelRotatorCommands();

        // Default Commands run whenever no Command is scheduled to run for a subsystem
        mClimber.setDefaultCommand(mClimberCommands.new Stop(mClimber));
        mIntake.setDefaultCommand(mIntakeCommands.new Stop(mIntake));
        mLauncher.setDefaultCommand(new ConditionalCommand(mLauncherCommands.new Target(mLauncher),
            mLauncherCommands.new Adjust(mLauncher), mLauncher::inRange));
        mPanelRotator.setDefaultCommand(mPanelRotatorCommands.new Stop(mPanelRotator));

        if(this.mJoystick != null)
            configureJoystickBindings();

        if(this.mButtonBoard != null)
            configureButtonBoardBindings();
    }

    private void configureJoystickBindings()
    {
        // NB: the need to keep a reference to a Joystick instance is
        // obviated by the fact that the CommandScheduler holds one.
        // In essence, the call to whenPressed and friends has
        // the side-effect of registering the JoystickButton instance with
        // CommandScheduler.

        // Note: changes to bling state can be augmented with:
        // .alongWith(new SetBlingStateCommand(mLED, BlingState.SOME_STATE)));

        /*
        ButtonFactory.Create(mJoystick, 1).whenPressed(() -> mDrive.driveSlow()).whenReleased(() -> mDrive.driveNormal());
        ButtonFactory.Create(mJoystick, 2).whenHeld(new LauncherCommands.Raise(mLauncher));
        ButtonFactory.Create(mJoystick, 3).whenHeld(new LauncherCommands.Lower(mLauncher));
        ButtonFactory.Create(mJoystick, 4).whenHeld(new LauncherCommands.Left(mLauncher));
        ButtonFactory.Create(mJoystick, 5).whenHeld(new LauncherCommands.Right(mLauncher));
        */

        /* Switch Camera views
        ButtonFactory.Create(mJoystick, 6).whenPressed(
            new InstantCommand(() -> mCamera.switch(Constants.Camera.kFrontId)));
        ButtonFactory.Create(mJoystick, 7).whenPressed(
            new InstantCommand(() -> mCamera.switch(Constants.Camera.kRearId)));
        ButtonFactory.Create(mJoystick, 10).whenPressed(
            new InstantCommand(() -> mCamera.switch(Constants.Camera.kIntakeId)));
        ButtonFactory.Create(mJoystick, 11).whenPressed(
            new InstantCommand(() -> mCamera.switch(Constants.Camera.kTurretId)));
        */

        // ButtonFactory.Create(mJoystick, 1).toggleWhenPressed(mLauncherCommands.new ShootBallTest(mLauncher));
        // ButtonFactory.Create(mJoystick, 2).toggleWhenPressed(mLauncherCommands.new TurretTest(mLauncher));
        // ButtonFactory.Create(mJoystick, 3).toggleWhenPressed(mLauncherCommands.new HoodTest(mLauncher));
        // ButtonFactory.Create(mJoystick, 7).whileHeld(new TrajectoryTrackerCommand(mDrive, mDrive,
        //    this::throughTrench, mRamseteController, mStateEstimator.getEncoderRobotStateMap()));
        // ButtonFactory.Create(mJoystick, 7).whileHeld(new TrajectoryTrackerCommand(mDrive, mDrive,
        //    this::toControlPanel, mRamseteController, mStateEstimator.getEncoderRobotStateMap()));
        // ButtonFactory.Create(mJoystick, 3).toggleWhenPressed(mLauncherCommands.new AutoAimTurret(mLauncher,Constants.Launcher.goalLocation,mStateEstimator.getEncoderRobotStateMap()));
    }

    private void configureButtonBoardBindings()
    {
        // XXX: ButtonId of 0 shouldn't exist?
        // ButtonFactory JoystickButton(mButtonBoard, 0).whenPressed(LauncherCommands.new Launch(mLauncher));
        // ButtonFactory JoystickButton(mButtonBoard, 1).toggleWhenPressed(new ConditionalCommand(mLauncherCommands.new Target));

        ButtonFactory.Create(mButtonBoard, 2).toggleWhenPressed(mIntakeCommands.new Harvest(mIntake));
        ButtonFactory.Create(mButtonBoard, 3).toggleWhenPressed(mIntakeCommands.new Eject(mIntake));

        ButtonFactory.Create(mButtonBoard, 4).whileHeld(mClimberCommands.new Retract(mClimber));
        ButtonFactory.Create(mButtonBoard, 5).whileHeld(mClimberCommands.new Extend(mClimber));

        ButtonFactory.Create(mButtonBoard, 6).whenPressed(mPanelRotatorCommands.new Raise(mPanelRotator));
        ButtonFactory.Create(mButtonBoard, 7).whenPressed(mPanelRotatorCommands.new Lower(mPanelRotator));
        ButtonFactory.Create(mButtonBoard, 8).whenPressed(mPanelRotatorCommands.new SpinOneRotation(mPanelRotator), false);
        ButtonFactory.Create(mButtonBoard, 9).whenPressed(mPanelRotatorCommands.new SpinToColor(mPanelRotator));

        ButtonFactory.Create(mButtonBoard, 10).whileHeld(mClimberCommands.new Extend(mClimber)
            .withTimeout(Constants.Climber.kTimerExtenderMin));
        ButtonFactory.Create(mButtonBoard, 11).whileHeld(mClimberCommands.new Extend(mClimber)
            .withTimeout(Constants.Climber.kTimerExtenderMax));

        ButtonFactory.Create(mButtonBoard, 12).whenPressed(new SequentialCommandGroup(
            mPanelRotatorCommands.new Raise(mPanelRotator),
            mPanelRotatorCommands.new SpinFourRotations(mPanelRotator),
            mPanelRotatorCommands.new Lower(mPanelRotator))); // TODO: will the act of lowering spin the wheel?

        ButtonFactory.Create(mButtonBoard, 13).whenPressed(new SequentialCommandGroup(
            mPanelRotatorCommands.new Raise(mPanelRotator),
            mPanelRotatorCommands.new SpinToColor(mPanelRotator),
            mPanelRotatorCommands.new Lower(mPanelRotator)));

        ButtonFactory.Create(mButtonBoard, 14).whenHeld(mClimberCommands.new WinchPrimary(mClimber)
            .andThen(mClimberCommands.new WinchSecondary(mClimber)));

        /* Four-way Joystick
        ButtonFactory.Create(mButtonBoard, 15).whenHeld(new TurretRaiseCommand(mLauncher));
        ButtonFactory.Create(mButtonBoard, 16).whenHeld(new TurretLowerCommand(mLauncher));
        ButtonFactory.Create(mButtonBoard, 17).whenHeld(new TurretLeftCommand(mLauncher));
        ButtonFactory.Create(mButtonBoard, 18).whenHeld(new TurretRightCommand(mLauncher));
        */
    }

    /**
     * configureTestCommands is not actually run. It is declared public to quell warnings.
     * Its use is to test out different construction idioms for externally defined commands.
     */
    public void configureTestCommands()
    {
        /**
         * in this style object construction happens in the CommandFactory
         * this.mExampleCommandFactory.MakeCmd(IndexerCommandFactory.CmdEnum.kTest1);
         *
         * in this mode we construct things here, we must pass in parameters
         * that are required during construction, since the outer class
         * member variables aren't accessible until after construction.
         * this.mExampleCommandFactory.new Test5(this.mLED); // an InstantCommand
         * this.mExampleCommandFactory.new Test6(this.mLED); // a StartEndCommand
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
        {
            new Pose2d(Units.inchesToMeters(424), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(207), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180))
        };
        for (int i = 0; i < intermediate.length; i++)
        {
            Pose2d pose = intermediate[i];
            intermediate[i] = new Pose2d(pose.getTranslation().getX() - Units.inchesToMeters(312.5),
                pose.getTranslation().getY(), pose.getRotation());
        }
        RobotStateMap stateMap = mStateEstimator.getEncoderRobotStateMap();
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

    public TimedTrajectory<Pose2dWithCurvature> toControlPanel()
    {
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d pose = mStateEstimator.getEncoderRobotStateMap().getLatestState().pose;
        waypoints.add(pose);
        Translation2d p = pose.getTranslation();
        Pose2d nextToControlPanel;
        if (p.getX() < Units.inchesToMeters(359))
        {
            nextToControlPanel = new Pose2d(Units.inchesToMeters(328), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(0));
        }
        else
        {
            nextToControlPanel = new Pose2d(Units.inchesToMeters(390), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180));
        }
        waypoints.add(nextToControlPanel);
        ArrayList<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<TimingConstraint<Pose2dWithCurvature>>();
        constraints.add(new VelocityLimitRegionConstraint(new Rectangle2d(
            new Translation2d(Units.inchesToMeters(290), Units.inchesToMeters(161.6)),
            new Translation2d(Units.inchesToMeters(428), Units.inchesToMeters(90))), .5));
        return TrajectoryContainer.generateTrajectory(waypoints, constraints);
    }
}
