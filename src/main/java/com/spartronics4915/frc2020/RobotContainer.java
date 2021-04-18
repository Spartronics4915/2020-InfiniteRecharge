package com.spartronics4915.frc2020;

import java.util.Arrays;
import java.util.stream.Collectors;

import com.spartronics4915.frc2020.commands.ClimberCommands;
import com.spartronics4915.frc2020.commands.DriveCommands;
import com.spartronics4915.frc2020.commands.IndexerCommands;
import com.spartronics4915.frc2020.commands.IntakeCommands;
import com.spartronics4915.frc2020.commands.LauncherCommands;
import com.spartronics4915.frc2020.commands.PanelRotatorCommands;
import com.spartronics4915.frc2020.commands.SuperstructureCommands;
import com.spartronics4915.frc2020.commands.LEDCommands;
import com.spartronics4915.frc2020.subsystems.Climber;
import com.spartronics4915.frc2020.subsystems.Drive;
import com.spartronics4915.frc2020.subsystems.Indexer;
import com.spartronics4915.frc2020.subsystems.Intake;
import com.spartronics4915.frc2020.subsystems.LED;
import com.spartronics4915.frc2020.subsystems.LED.Bling;
import com.spartronics4915.frc2020.subsystems.Launcher;
import com.spartronics4915.frc2020.subsystems.PanelRotator;
import com.spartronics4915.frc2020.subsystems.Vision;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.hardware.sensors.T265Camera;
import com.spartronics4915.lib.hardware.sensors.T265Camera.CameraJNIException;
import com.spartronics4915.lib.math.twodim.control.RamseteTracker;
import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.subsystems.estimator.DrivetrainEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator.EstimatorSource;
import com.spartronics4915.lib.util.Kinematics;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotContainer
{
    private static final String kAutoOptionsKey = "AutoStrategyOptions";
    public final NetworkTableEntry mAutoModeEntry = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard").getEntry("AutoStrategy");

    /* subsystems, public for easier unit testing */
    public final Climber mClimber;
    public final Intake mIntake;
    public final Indexer mIndexer;
    public final Launcher mLauncher;
    public final PanelRotator mPanelRotator;
    public final LED mLED;
    public final Vision mVision;
    public final Drive mDrive;
    public final TrajectoryTracker mRamseteController = new RamseteTracker(2, 0.7);
    public final RobotStateEstimator mStateEstimator;
    public final TrajectoryContainer.AutoMode[] mAutoModes;

    /* subsystem commands, public for easier unit testing */
    public final LEDCommands mLEDCommands;
    public final ClimberCommands mClimberCommands;
    public final DriveCommands mDriveCommands;
    public final IntakeCommands mIntakeCommands;
    public final IndexerCommands mIndexerCommands;
    public final LauncherCommands mLauncherCommands;
    public final PanelRotatorCommands mPanelRotatorCommands;
    public final SuperstructureCommands mSuperstructureCommands;

    private final ButtonFactory mButtons;
    private final int mJoystick = Constants.OI.kJoystickId;
    private final int mButtonBoard = Constants.OI.kButtonBoardId;;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        int retryCount = 0;
        T265Camera slamra = null;
        while (++retryCount <= 1 && slamra == null)
        {
            try
            {
                slamra = new T265Camera(Constants.Estimator.kSlamraToRobot,
                    Constants.Estimator.kT265InternalMeasurementCovariance);
                SmartDashboard.putString("RobotContainer/vslamStatus", "OK");
            }
            catch (CameraJNIException | UnsatisfiedLinkError e)
            {
                slamra = null;
                Logger.error("RobotContainer: T265 camera is unavailable");
                Logger.exception(e);
                SmartDashboard.putString("RobotContainer/vslamStatus", "BAD!");
            }
        }

        mButtons = new ButtonFactory();

        /* constructing subsystems */
        mClimber = new Climber();
        mIntake = new Intake();
        mIndexer = new Indexer();
        mLauncher = new Launcher();
        mPanelRotator = new PanelRotator();
        mLED = LED.getInstance();
        mDrive = new Drive(mLauncher);

        var ekf = new DrivetrainEstimator(
            mDrive.getIMUHeading(),
            Constants.Trajectory.kStartPointRight,
            Constants.Estimator.kStateStdDevs,
            Constants.Estimator.kLocalMeasurementStdDevs,
            Constants.Estimator.kVisionMeasurementStdDevs
        );
        mStateEstimator = new RobotStateEstimator(mDrive,
            new Kinematics(Constants.Drive.kTrackWidthMeters, Constants.Drive.kScrubFactor),
            slamra,
            ekf,
            slamra == null ? EstimatorSource.EncoderOdometry : EstimatorSource.Fused);
        StartEndCommand slamraCmd = new StartEndCommand(
            () -> mStateEstimator.enable(),
            () -> mStateEstimator.stop(),
            mStateEstimator);
        mStateEstimator.setDefaultCommand(slamraCmd);
        mStateEstimator.resetRobotStateMaps(Constants.Trajectory.kStartPointAtHome); // NOTE: This is configured for 2021
        mVision = new Vision(mStateEstimator);

        if (!RobotBase.isReal()) // we're unit testing
            SpartronicsSimulatedMotor.resetGlobalState();

        /* constructing subsystem commands */
        mLEDCommands = new LEDCommands(mLED);
        mClimberCommands = new ClimberCommands(mClimber);
        mDriveCommands = new DriveCommands(mDrive, mButtons.getJoystick(mJoystick));
        mIntakeCommands = new IntakeCommands(mIntake, mIndexer);
        mIndexerCommands = new IndexerCommands(mIndexer);
        mLauncherCommands = new LauncherCommands(mLauncher, mVision, mIndexerCommands,
            mStateEstimator.getBestRobotStateMap());
        mPanelRotatorCommands = new PanelRotatorCommands(mPanelRotator);
        mSuperstructureCommands = new SuperstructureCommands(mIndexerCommands,
            mIntakeCommands, mLauncherCommands);

        mVision.registerTargetListener(mStateEstimator.getVisionListener());

        // NB: ButtonFactory handles the !RobotBase.isReal case.
        // configureJoystickBindings();
        // configureButtonBoardBindings();
        configureSingleJoystickBindings();

        /* publish our automodes to the dashboard -----------------*/
        mAutoModes = TrajectoryContainer.getAutoModes(mStateEstimator, mDrive, mRamseteController,
            mSuperstructureCommands);
        String autoModeList = Arrays.stream(mAutoModes).map((m) -> m.name)
            .collect(Collectors.joining(","));
        SmartDashboard.putString(kAutoOptionsKey, autoModeList);
    }

    // Joystick buttons are labelled on the joystick! what a concept
    private void configureJoystickBindings()
    {
        // toggle animation to indicate SLOW vs NORMAL drive speeds
        mButtons.create(mJoystick, 1)
            .whenPressed(mDriveCommands.new SetSlow()
                .alongWith(mLEDCommands.new SetBlingState(Bling.kDriveSlow)))
            .whenReleased(mDriveCommands.new ToggleSlow()
                .alongWith(mLEDCommands.new SetBlingState(Bling.kTeleop)));

        // Chris has expressed he doesn't want functionality on buttons 2, 4, and 5
        mButtons.create(mJoystick, 3).whenPressed(mDriveCommands.new ToggleInverted());

        // JoystickButton 6 / 7 have the same functionality - they're close together + on passive hand side
        mButtons.create(mJoystick, 6).whenPressed(mDriveCommands.new ToggleSlow()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kDriveSlow))); // FIXME: this will not go back to kTeleop
        mButtons.create(mJoystick, 7).whenPressed(mDriveCommands.new ToggleSlow()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kDriveSlow)));

        mButtons.create(mJoystick, 10).whenPressed(new InstantCommand(() -> mIndexer.setZero()));
        mButtons.create(mJoystick, 11)
            .whenPressed(new InstantCommand(() -> mLauncher.zeroTurret()));
    }

    /**
     * +--------------------------------------+
     * |               (9)   (7)(8)  (10)(11) |
     * |                                      |
     * |     (x)            (3)   (4)         |
     * |  (x)++(x)     (5)             (6)    |
     * |    (x)            (1)   (2)          |
     * |             (x)              (x)     |
     * |                                      |
     * +--------------------------------------+
     */
    private void configureButtonBoardBindings()
    {
        // launch buttons
        mButtons.create(mButtonBoard, 4).whenPressed(mSuperstructureCommands.new LaunchSequence(1)
            .alongWith(mLEDCommands.new SetBlingState(Bling.kLaunch)));
        mButtons.create(mButtonBoard, 3).whenPressed(mSuperstructureCommands.new LaunchSequence(5))
            .whileActiveContinuous(mLEDCommands.new SetBlingState(Bling.kLaunch));

        // toggle pickup (command group)
        mButtons.create(mButtonBoard, 2).toggleWhenPressed(mSuperstructureCommands.new IntakeFive())
            .whenActive(mLEDCommands.new SetBlingState(Bling.kIntake))
            .whenInactive(mLEDCommands.new SetBlingState(Bling.kTeleop));

        // toggle eject
        mButtons.create(mButtonBoard, 1).toggleWhenPressed(mIntakeCommands.new Eject())
            .whenActive(mLEDCommands.new SetBlingState(Bling.kEject))
            .whenInactive(mLEDCommands.new SetBlingState(Bling.kTeleop));

        // climb buttons
        mButtons.create(mButtonBoard, 8).whenHeld(mClimberCommands.new Winch()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));
        mButtons.create(mButtonBoard, 9).whileHeld(mClimberCommands.new Retract()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));
        mButtons.create(mButtonBoard, 10).whileHeld(mClimberCommands.new Extend()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));
        mButtons.create(mButtonBoard, 10).whenPressed(mLauncherCommands.new SetAsideToClimb());

        // control panel buttons - turning off LEDs to minimize interference
        mButtons.create(mButtonBoard, 5).whenPressed(mPanelRotatorCommands.new Lower()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kTeleop)));
        mButtons.create(mButtonBoard, 6).whenPressed(mPanelRotatorCommands.new Raise()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kOff)));
        mButtons.create(mButtonBoard, 7).whenPressed(mPanelRotatorCommands.new SpinToColor());
    }

    /**
     * for when you don't have the button board to test commands
     * buttons are numbered
     */
    public void configureSingleJoystickBindings()
    {
        mButtons.create(mJoystick, 1)
            .whenPressed(mDriveCommands.new SetSlow()
                .alongWith(mLEDCommands.new SetBlingState(Bling.kDriveSlow)))
            .whenReleased(mDriveCommands.new ToggleSlow()
                .alongWith(mLEDCommands.new SetBlingState(Bling.kTeleop)));

        // toggle intake and eject
        mButtons.create(mJoystick, 2).toggleWhenPressed(mIntakeCommands.new Eject())
            .whenActive(mLEDCommands.new SetBlingState(Bling.kEject))
            .whenInactive(mLEDCommands.new SetBlingState(Bling.kTeleop));
        mButtons.create(mJoystick, 3).toggleWhenPressed(mSuperstructureCommands.new IntakeFive())
            .whenActive(mLEDCommands.new SetBlingState(Bling.kIntake))
            .whenInactive(mLEDCommands.new SetBlingState(Bling.kTeleop));

        // launch buttons
        mButtons.create(mJoystick, 4).whenPressed(mSuperstructureCommands.new LaunchSequence(1)
            .alongWith(mLEDCommands.new SetBlingState(Bling.kLaunch)));
        mButtons.create(mJoystick, 5).whenPressed(mSuperstructureCommands.new LaunchSequence(5))
            .whileActiveContinuous(mLEDCommands.new SetBlingState(Bling.kLaunch));

        /*
        // control panel buttons- turning off LEDs to minimize interference
        mButtons.create(mJoystick, 6).whenPressed(mPanelRotatorCommands.new Lower()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kTeleop)));
        mButtons.create(mJoystick, 7).whenPressed(mPanelRotatorCommands.new Raise()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kOff)));
        mButtons.create(mJoystick, 8).whenPressed(mPanelRotatorCommands.new SpinToColor());

        // climber buttons
        mButtons.create(mJoystick, 9).whenPressed(mClimberCommands.new Winch()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));
        mButtons.create(mJoystick, 10).whileHeld(mClimberCommands.new Retract()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));
        mButtons.create(mJoystick, 11).whileHeld(mClimberCommands.new Extend()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));
        */

        // localization reset buttons
        mButtons.create(mJoystick, 6).whenPressed(new InstantCommand(() ->
            mStateEstimator.resetRobotStateMaps(Constants.Trajectory.kGreenZoneMiddle)));
        mButtons.create(mJoystick, 7).whenPressed(new InstantCommand(() ->
            mStateEstimator.resetRobotStateMaps(Constants.Trajectory.kYellowZoneMiddle)));
        mButtons.create(mJoystick, 10).whenPressed(new InstantCommand(() ->
            mStateEstimator.resetRobotStateMaps(Constants.Trajectory.kBlueZoneMiddle)));
        mButtons.create(mJoystick, 11).whenPressed(new InstantCommand(() ->
            mStateEstimator.resetRobotStateMaps(Constants.Trajectory.kRedZoneMiddle)));
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
