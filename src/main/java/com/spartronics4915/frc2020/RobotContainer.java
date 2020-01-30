package com.spartronics4915.frc2020;

import java.util.Set;

import com.spartronics4915.frc2020.commands.*;
import com.spartronics4915.frc2020.subsystems.Climber;
import com.spartronics4915.frc2020.subsystems.Launcher;
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
            return null;
        }
    });

    public final AutoMode[] mAutoModes;

    private Joystick mJoystick = new Joystick(Constants.OI.kJoystickId);
    private Joystick mButtonBoard = new Joystick(Constants.OI.kButtonBoardId);
    private Climber mClimber;
    private ClimberCommands mClimberCommands;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        configureJoystickBindings();
        configureButtonBoardBindings();
        mAutoModes = new AutoMode[]
        {kDefaultAutoMode,};
        mClimber = new Climber();
        mClimberCommands = new ClimberCommands(mClimber);

    }

    private void configureJoystickBindings()
    {
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
        new JoystickButton(mJoystick, 2).whileHeld(mClimberCommands.new Extend());
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
}
