package com.spartronics4915.frc2020;

import java.util.Set;

import com.spartronics4915.frc2020.TrajectoryContainer.Destination;
import com.spartronics4915.frc2020.commands.*;
import com.spartronics4915.lib.hardware.sensors.T265Camera;
import com.spartronics4915.lib.hardware.sensors.T265Camera.CameraJNIException;
import com.spartronics4915.lib.math.twodim.control.RamseteTracker;
import com.spartronics4915.lib.subsystems.drive.TrajectoryTrackerCommand;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.util.Kinematics;
import com.spartronics4915.frc2020.subsystems.*;
import com.spartronics4915.frc2020.subsystems.LED.BlingState;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public static final String kSelectedAutoModeKey = "AutoStrategy";
    public static final AutoMode kDefaultAutoMode = new AutoMode("All: Do Nothing", new Command()
    {
        @Override
        public Set<Subsystem> getRequirements()
        {
            return Set.of();
        }
    });

    public final AutoMode[] mAutoModes;

    private final Climber mClimber;
    private final Intake mIntake;
    private final Launcher mLauncher;
    private final PanelRotator mPanelRotator;
    private final LED mLED;
    private final ClimberCommands mClimberCommands;
    private final PanelRotatorCommands mPanelRotatorCommands;

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
        mClimber = new Climber();
        mIntake = new Intake();
        mLauncher = new Launcher();
        mPanelRotator = new PanelRotator();
        mLED = LED.getInstance();
        mClimberCommands = new ClimberCommands();
        mPanelRotatorCommands = new PanelRotatorCommands();

        mJoystick = new Joystick(Constants.OI.kJoystickId);
        mButtonBoard = new Joystick(Constants.OI.kButtonBoardId);

        configureJoystickBindings();
        configureButtonBoardBindings();

        mDrive = new Drive();

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
        mStateEstimator = new RobotStateEstimator(mDrive,
            new Kinematics(Constants.Drive.kTrackWidthMeters, Constants.Drive.kScrubFactor),
            slamra);
        mAutoModes = new AutoMode[] {kDefaultAutoMode,
            new AutoMode("drive straight",
                new TrajectoryTrackerCommand(mDrive,
                    TrajectoryContainer.middle.getTrajectory(Destination.backOfShieldGenerator),
                    mRamseteController, mStateEstimator.getCameraRobotStateMap()))};
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
        new JoystickButton(mJoystick, 1).toggleWhenPressed(new ShootBallTest(mLauncher));
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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        String selectedModeName = SmartDashboard.getString(kSelectedAutoModeKey,
            "NO SELECTED MODE!!!!");
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

    /**
     * Sets bling state -- used by robot.java code
     * TODO: verify this is how we want to interface to disabledInit()
    */
    public void setBlingState(BlingState blingState)
    {
        mLED.setBlingState(blingState);
    }
}
