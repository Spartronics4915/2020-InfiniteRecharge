package com.spartronics4915.frc2020;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

import com.spartronics4915.frc2020.TrajectoryContainer.Destination;
import com.spartronics4915.frc2020.commands.*;
import com.spartronics4915.frc2020.subsystems.*;
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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer
{
    private static final String kAutoOptionsKey = "AutoStrategyOptions";

    public final NetworkTableEntry mAutoModeEntry = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard").getEntry("AutoStrategy");

    /* subsystems */
    private final Climber mClimber;
    private final Intake mIntake;
    private final Indexer mIndexer;
    private final Launcher mLauncher;
    private final PanelRotator mPanelRotator;
    private final LED mLED;
    private final Vision mVision;
    private final Drive mDrive;
    private final RamseteTracker mRamseteController = new RamseteTracker(2, 0.7);
    private final RobotStateEstimator mStateEstimator;
    private final CoordSysMgr2020 mCamToField;
    private final TrajectoryContainer.AutoMode[] mAutoModes;

    /* subsystem commands */
    private final ClimberCommands mClimberCommands;
    private final DriveCommands mDriveCommands;
    private final IntakeCommands mIntakeCommands;
    private final IndexerCommands mIndexerCommands;
    private final LauncherCommands mLauncherCommands;
    private final PanelRotatorCommands mPanelRotatorCommands;
    private final SuperstructureCommands mSuperstructureCommands;

    private final Joystick mJoystick;
    private final Joystick mButtonBoard;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
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
        mStateEstimator.resetRobotStateMaps(new Pose2d());
        mCamToField = new CoordSysMgr2020();

        mAutoModes = TrajectoryContainer.getAutoModes(mStateEstimator, mDrive, mRamseteController);
        String autoModeList = Arrays.stream(mAutoModes).map((m) -> m.name)
            .collect(Collectors.joining(","));
        SmartDashboard.putString(kAutoOptionsKey, autoModeList);

        mJoystick = new Joystick(Constants.OI.kJoystickId);
        mButtonBoard = new Joystick(Constants.OI.kButtonBoardId);

        /* constructing subsystems */
        mClimber = new Climber();
        mIntake = new Intake();
        mIndexer = new Indexer();
        mLauncher = new Launcher();
        mPanelRotator = new PanelRotator();
        mLED = LED.getInstance();
        mVision = new Vision(mStateEstimator, mCamToField, mLauncher);

        /* constructing subsystem commands */
        mClimberCommands = new ClimberCommands(mClimber);
        mDriveCommands = new DriveCommands(mDrive, mJoystick);
        mIntakeCommands = new IntakeCommands(mIntake);
        mIndexerCommands = new IndexerCommands(mIndexer);
        mLauncherCommands = new LauncherCommands(mLauncher, mIndexerCommands,
                                mStateEstimator.getEncoderRobotStateMap(),
                                mCamToField);
        mPanelRotatorCommands = new PanelRotatorCommands(mPanelRotator);
        mSuperstructureCommands = new SuperstructureCommands(mIndexerCommands, 
                                            mIntakeCommands, mLauncherCommands);

        // Default Commands run whenever no Command is scheduled to run for a subsystem
        mClimber.setDefaultCommand(mClimberCommands.new Stop());
        mIntake.setDefaultCommand(mIntakeCommands.new Stop());
        // mLauncher.setDefaultCommand(new ConditionalCommand(mLauncherCommands.new TargetAndShoot(mLauncher),
        //     mLauncherCommands.new TrackPassively(mLauncher), mLauncher::inRange));
        mLauncher.setDefaultCommand(mLauncherCommands.new ShootBallTest());//mLauncherCommands.new TargetAndShoot(mLauncher));
        mPanelRotator.setDefaultCommand(mPanelRotatorCommands.new Stop());
        mDrive.setDefaultCommand(mDriveCommands.new TeleOpCommand());


        // mLauncherCommands.new Zero(mLauncher).schedule();
        configureJoystickBindings();
        configureButtonBoardBindings();
    }

    private void configureJoystickBindings()
    {
        // Note: changes to bling state can be augmented with:
        // .alongWith(new SetBlingStateCommand(mLED, BlingState.SOME_STATE)));

        /*
        new JoystickButton(mJoystick, 1).whenPressed(mIndexerCommands.new ZeroSpinnerCommand(true));
        new JoystickButton(mJoystick, 2).whenPressed(mIndexerCommands.new SpinIndexer(5));
        new JoystickButton(mJoystick, 4).whenPressed(mIndexerCommands.new StartTransfer())
            .whenReleased(mIndexerCommands.new EndTransfer());
        new JoystickButton(mJoystick, 5).whenPressed(mIndexerCommands.new StartKicker())
            .whenReleased(mIndexerCommands.new EndKicker());
        new JoystickButton(mJoystick, 6).whenPressed(mSuperstructureCommands.new LaunchSequence());
        */

        /*
        new JoystickButton(mJoystick, 1).toggleWhenPressed(mLauncherCommands.new ShootBallTest());
        new JoystickButton(mJoystick, 2).toggleWhenPressed(mLauncherCommands.new Zero());
        new JoystickButton(mJoystick, 3).toggleWhenPressed(mLauncherCommands.new HoodTest());
        new JoystickButton(mJoystick, 4).toggleWhenPressed(mPanelRotatorCommands.new Raise());
        new JoystickButton(mJoystick, 5).toggleWhenPressed(mPanelRotatorCommands.new Lower());
        new JoystickButton(mJoystick, 6).toggleWhenPressed(mPanelRotatorCommands.new SpinToColor());
        */

        /* Test Command that fires all balls after setting Flywheel/Hood values from SmartDashboard
        new JoystickButton(mJoystick, 4).toggleWhenPressed(new SequentialCommandGroup(
            new ParallelRaceGroup(
                new ParallelCommandGroup(mIndexerCommands.new SpinUpKicker(mIndexer),
                    mLauncherCommands.new ShootBallTest(mLauncher)),
                mLauncherCommands.new WaitForFlywheel(mLauncher)),
            new ParallelCommandGroup(mIndexerCommands.new LoadToLauncher(mIndexer, 5),
                mLauncherCommands.new ShootBallTest(mLauncher))));
        new JoystickButton(mJoystick, 5).toggleWhenPressed(new SequentialCommandGroup(
            new ParallelRaceGroup(
                new ParallelCommandGroup(mIndexerCommands.new SpinUpKicker(mIndexer),
                    mLauncherCommands.new ShootBallTestWithDistance(mLauncher)),
                mLauncherCommands.new WaitForFlywheel(mLauncher)),
            new ParallelCommandGroup(mIndexerCommands.new LoadToLauncher(mIndexer, 5),
                mLauncherCommands.new ShootBallTestWithDistance(mLauncher))));
        */

        /*
        new JoystickButton(mJoystick, 7).whileHeld(new TrajectoryTrackerCommand(mDrive, mDrive,
            this::throughTrench, mRamseteController, mStateEstimator.getEncoderRobotStateMap()));
        new JoystickButton(mJoystick, 7).whileHeld(new TrajectoryTrackerCommand(mDrive, mDrive,
            this::toControlPanel, mRamseteController, mStateEstimator.getEncoderRobotStateMap()));
        new JoystickButton(mJoystick, 3).toggleWhenPressed(mLauncherCommands.new AutoAimTurret(
            mLauncher,Constants.Launcher.goalLocation,mStateEstimator.getEncoderRobotStateMap()));
        */
    }

    private void configureButtonBoardBindings()
    {
        new JoystickButton(mButtonBoard, 1).whenHeld(mClimberCommands.new Winch());
        new JoystickButton(mButtonBoard, 2).whileHeld(mClimberCommands.new Extend());
        new JoystickButton(mButtonBoard, 3).whileHeld(mClimberCommands.new Retract());

        new JoystickButton(mButtonBoard, 4).toggleWhenPressed(mIntakeCommands.new Eject());
        new JoystickButton(mButtonBoard, 5).toggleWhenPressed(mSuperstructureCommands.new IntakeRace());

        // new JoystickButton(mButtonBoard, 6).toggleWhenPressed(new ConditionalCommand(mLauncherCommands.new Target());
        // new JoystickButton(mButtonBoard, 7).whenPressed(LauncherCommands.new Launch());

        new JoystickButton(mButtonBoard, 8).whenPressed(mPanelRotatorCommands.new Lower());
        new JoystickButton(mButtonBoard, 9).whenPressed(mPanelRotatorCommands.new Raise());
        new JoystickButton(mButtonBoard, 10).whenPressed(mPanelRotatorCommands.new SpinRotation(), false);
        new JoystickButton(mButtonBoard, 11).whenPressed(mPanelRotatorCommands.new SpinToColor());

        // TODO: interface with the button board "joystick" potentially through GenericHID
        // new JoystickButton(mButtonBoard, 12).whenPressed(mClimberCommands.new ExtendMin());
        // new JoystickButton(mButtonBoard, 13).whenPressed(mClimberCommands.new ExtendMax());
        // new JoystickButton(mButtonBoard, 14).whenPressed(mPanelRotatorCommands.new AutoSpinRotation());
        // new JoystickButton(mButtonBoard, 15).whenPressed(mPanelRotatorCommands.new AutoSpinToColor());

        /* Four-way Joystick
        new JoystickButton(mButtonBoard, 15).whenHeld(new TurretRaiseCommand());
        new JoystickButton(mButtonBoard, 16).whenHeld(new TurretLowerCommand());
        new JoystickButton(mButtonBoard, 17).whenHeld(new TurretLeftCommand());
        new JoystickButton(mButtonBoard, 18).whenHeld(new TurretRightCommand());
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
        return TrajectoryContainer.kDefaultAutoMode.command;
    }
}
