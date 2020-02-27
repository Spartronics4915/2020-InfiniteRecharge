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
import com.spartronics4915.lib.math.twodim.control.FeedForwardTracker;
import com.spartronics4915.lib.math.twodim.control.RamseteTracker;
import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.estimator.DrivetrainEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator.EstimatorSource;
import com.spartronics4915.lib.util.Kinematics;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Units;

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
    public final TrajectoryTracker mRamseteController = new FeedForwardTracker();
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
        T265Camera slamra;
        try
        {
            slamra = new T265Camera(Constants.Estimator.kSlamraToRobot,
                Constants.Estimator.kT265InternalMeasurementCovariance);
        }
        catch (CameraJNIException | UnsatisfiedLinkError e)
        {
            slamra = null;
            Logger.warning("RobotContainer: T265 camera is unavailable");
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

        var ekf = new DrivetrainEstimator(Constants.Estimator.kStateStdDevs,
            Constants.Estimator.measurementStdDevs, Constants.Estimator.kSlamStdDevsPerMeter,
            Constants.Estimator.kApproximateStartingPose);
        mStateEstimator = new RobotStateEstimator(mDrive,
            new Kinematics(Constants.Drive.kTrackWidthMeters, Constants.Drive.kScrubFactor),
            slamra,
            ekf,
            EstimatorSource.Fused);
        StartEndCommand slamraCmd = new StartEndCommand(
            () -> mStateEstimator.enable(),
            () -> mStateEstimator.stop(),
            mStateEstimator);
        mStateEstimator.setDefaultCommand(slamraCmd);
        mStateEstimator.resetRobotStateMaps(new Pose2d());
        mVision = new Vision(mStateEstimator, mLauncher);

        if (!RobotBase.isReal()) // we're unit testing
            SpartronicsSimulatedMotor.resetGlobalState();

        /* constructing subsystem commands */
        mLEDCommands = new LEDCommands(mLED);
        mClimberCommands = new ClimberCommands(mClimber);
        mDriveCommands = new DriveCommands(mDrive, mButtons.getJoystick(mJoystick));
        mIntakeCommands = new IntakeCommands(mIntake, mIndexer);
        mIndexerCommands = new IndexerCommands(mIndexer);
        mLauncherCommands = new LauncherCommands(mLauncher, mIndexerCommands,
            mStateEstimator.getEncoderRobotStateMap());
        mPanelRotatorCommands = new PanelRotatorCommands(mPanelRotator);
        mSuperstructureCommands = new SuperstructureCommands(mIndexerCommands,
            mIntakeCommands, mLauncherCommands);

        // Default Commands are established in each commands class

        // NB: ButtonFactory handles the !RobotBase.isReal case.
        configureJoystickBindings();
        configureButtonBoardBindings();

        /* publish our automodes to the dashboard -----------------*/
        mAutoModes = TrajectoryContainer.getAutoModes(mStateEstimator, mDrive, mRamseteController, mSuperstructureCommands);
        String autoModeList = Arrays.stream(mAutoModes).map((m) -> m.name)
            .collect(Collectors.joining(","));
        SmartDashboard.putString(kAutoOptionsKey, autoModeList);
        
    }

    private void configureJoystickBindings()
    {
        /* toggle animation to indicate SLOW vs NORMAL drive speeds */
        mButtons.create(mJoystick, 1).whenPressed(mDriveCommands.new SetSlow()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kDriveSlow)))
            .whenReleased(mDriveCommands.new ToggleSlow()
                .alongWith(mLEDCommands.new SetBlingState(Bling.kTeleop)));
        mButtons.create(mJoystick, 8).whenPressed(new InstantCommand(() -> mIndexer.setZero()));
        mButtons.create(mJoystick, 10)
            .whenPressed(new InstantCommand(() -> mIndexer.setZero()));
        // .whenPressed(mIndexerCommands.new ZeroSpinnerCommand(true));
        mButtons.create(mJoystick, 11)
            .whenPressed(new InstantCommand(() -> mLauncher.zeroTurret()));

        // Chris has expressed he doesn't want functionality on buttons 2, 4, and 5
        mButtons.create(mJoystick, 3).whenPressed(mDriveCommands.new ToggleInverted()); // TODO:
                                                                                        // alongWith
                                                                                        // Vision

        // Both JoystickButton 6 and 7 have the same functionality - they're close
        // together + on passive hand side
        /* animation for drive SLOW */
        mButtons.create(mJoystick, 6).whenPressed(mDriveCommands.new ToggleSlow() // TODO: alongWith
                                                                                  // Vision
            .alongWith(mLEDCommands.new SetBlingState(Bling.kDriveSlow)));
        mButtons.create(mJoystick, 7).whenPressed(mDriveCommands.new ToggleSlow() // TODO: alongWith
                                                                                  // Vision
            .alongWith(mLEDCommands.new SetBlingState(Bling.kDriveSlow)));

        mButtons.create(mJoystick, 2).whenPressed(mIndexerCommands.new SpinIndexer(5));
        mButtons.create(mJoystick, 4).whenPressed(mIndexerCommands.new StartTransfer())
            .whenReleased(mIndexerCommands.new EndTransfer());
        mButtons.create(mJoystick, 5).whenPressed(mIndexerCommands.new StartKicker())
            .whenReleased(mIndexerCommands.new EndKicker());
        /*
        mButtons.create(mJoystick, 1).whenPressed(mIndexerCommands.new ZeroSpinnerCommand(true));
        mButtons.create(mJoystick, 2).whenPressed(mIndexerCommands.new SpinIndexer(5));
        mButtons.create(mJoystick, 4).whenPressed(mIndexerCommands.new StartTransfer())
            .whenReleased(mIndexerCommands.new EndTransfer());
        mButtons.create(mJoystick, 5).whenPressed(mIndexerCommands.new StartKicker())
            .whenReleased(mIndexerCommands.new EndKicker());
        mButtons.create(mJoystick, 6).whenPressed(mSuperstructureCommands.new LaunchSequence());
        */

        /*
        mButtons.create(mJoystick, 1).toggleWhenPressed(mLauncherCommands.new ShootBallTest());
        mButtons.create(mJoystick, 2).toggleWhenPressed(mLauncherCommands.new Zero());
        mButtons.create(mJoystick, 3).toggleWhenPressed(mLauncherCommands.new HoodTest());
        mButtons.create(mJoystick, 4).toggleWhenPressed(mPanelRotatorCommands.new Raise());
        mButtons.create(mJoystick, 5).toggleWhenPressed(mPanelRotatorCommands.new Lower());
        mButtons.create(mJoystick, 6).toggleWhenPressed(mPanelRotatorCommands.new SpinToColor());
        */

        /* Test Command that fires all balls after setting Flywheel/Hood values from SmartDashboard
        mButtons.create(mJoystick, 4).toggleWhenPressed(new SequentialCommandGroup(
            new ParallelRaceGroup(
                new ParallelCommandGroup(mIndexerCommands.new SpinUpKicker(mIndexer),
                    mLauncherCommands.new ShootBallTest(mLauncher)),
                mLauncherCommands.new WaitForFlywheel(mLauncher)),
            new ParallelCommandGroup(mIndexerCommands.new LoadToLauncher(mIndexer, 5),
                mLauncherCommands.new ShootBallTest(mLauncher))));
        mButtons.create(mJoystick, 5).toggleWhenPressed(new SequentialCommandGroup(
            new ParallelRaceGroup(
                new ParallelCommandGroup(mIndexerCommands.new SpinUpKicker(mIndexer),
                    mLauncherCommands.new ShootBallTestWithDistance(mLauncher)),
                mLauncherCommands.new WaitForFlywheel(mLauncher)),
            new ParallelCommandGroup(mIndexerCommands.new LoadToLauncher(mIndexer, 5),
                mLauncherCommands.new ShootBallTestWithDistance(mLauncher))));
        */

        /*
        mButtons.create(mJoystick, 7).whileHeld(new TrajectoryTrackerCommand(mDrive, mDrive,
            this::throughTrench, mRamseteController, mStateEstimator.getEncoderRobotStateMap()));
        mButtons.create(mJoystick, 7).whileHeld(new TrajectoryTrackerCommand(mDrive, mDrive,
            this::toControlPanel, mRamseteController, mStateEstimator.getEncoderRobotStateMap()));
        mButtons.create(mJoystick, 3).toggleWhenPressed(mLauncherCommands.new AutoAimTurret(
            mLauncher,Constants.Launcher.goalLocation,mStateEstimator.getEncoderRobotStateMap()));
        */
    }

    private void configureButtonBoardBindings()
    {
        /* animate launch */
        mButtons.create(mButtonBoard, 4).whenPressed(mSuperstructureCommands.new LaunchSequence(1)
            .alongWith(mLEDCommands.new SetBlingState(Bling.kLaunch)));
        /* TODO: validate multiple launch animations */
        mButtons.create(mButtonBoard, 3).whenPressed(mSuperstructureCommands.new LaunchSequence(5))
            .whileActiveContinuous(mLEDCommands.new SetBlingState(Bling.kLaunch));
        /* animation for pickup: change bling state when command active/inactive */
        // TODO: validate pickup animation
        mButtons.create(mButtonBoard, 2).toggleWhenPressed(mSuperstructureCommands.new IntakeFive())
            .whenActive(mLEDCommands.new SetBlingState(Bling.kIntake))
            .whenInactive(mLEDCommands.new SetBlingState(Bling.kTeleop));
        /* animation for eject: change bling state when command active/inactive */
        // TODO: validate eject animation
        mButtons.create(mButtonBoard, 1).toggleWhenPressed(mIntakeCommands.new Eject())
            .whenActive(mLEDCommands.new SetBlingState(Bling.kEject))
            .whenInactive(mLEDCommands.new SetBlingState(Bling.kTeleop));

        /* animation for climb -- note, we are not differentiating different climb states */
        mButtons.create(mButtonBoard, 8).whenHeld(mClimberCommands.new Winch()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));
        mButtons.create(mButtonBoard, 9).whileHeld(mClimberCommands.new Retract()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));
        mButtons.create(mButtonBoard, 10).whileHeld(mClimberCommands.new Extend()
            .alongWith(mLEDCommands.new SetBlingState(Bling.kClimb)));

        // new JoystickButton(mButtonBoard, 6).toggleWhenPressed(new
        // ConditionalCommand(mLauncherCommands.new Target());
        // new JoystickButton(mButtonBoard, 7).whenPressed(LauncherCommands.new
        // Launch());

        /* turning off LEDs for control panel actions to minimize interference */
        // mButtons.create(mButtonBoard, 5).whenPressed(mPanelRotatorCommands.new
        // Lower()
        // .alongWith(mLEDCommands.new SetBlingState(Bling.kTeleop)));
        // mButtons.create(mButtonBoard, 6).whenPressed(mPanelRotatorCommands.new
        // Raise()
        // .alongWith(mLEDCommands.new SetBlingState(Bling.kOff)));
        // mButtons.create(mButtonBoard, 7).whenPressed(mPanelRotatorCommands.new
        // SpinToColor());

        // TODO: interface with the button board "joystick" potentially through
        // GenericHID
        // mButtons.create(mButtonBoard, 12).whenPressed(mClimberCommands.new
        // ExtendMin());
        // mButtons.create(mButtonBoard, 13).whenPressed(mClimberCommands.new
        // ExtendMax());
        // mButtons.create(mButtonBoard, 14).whenPressed(mPanelRotatorCommands.new
        // AutoSpinRotation());
        // mButtons.create(mButtonBoard, 15).whenPressed(mPanelRotatorCommands.new
        // AutoSpinToColor());

        /* Four-way Joystick
        mButtons.create(mButtonBoard, 15).whenHeld(new TurretRaiseCommand());
        mButtons.create(mButtonBoard, 16).whenHeld(new TurretLowerCommand());
        mButtons.create(mButtonBoard, 17).whenHeld(new TurretLeftCommand());
        mButtons.create(mButtonBoard, 18).whenHeld(new TurretRightCommand());
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
