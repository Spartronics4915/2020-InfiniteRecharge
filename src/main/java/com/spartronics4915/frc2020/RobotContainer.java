package com.spartronics4915.frc2020;

import java.util.Set;

import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RobotContainer {

  private static class AutoMode {
    public final String name;
    public final Command command;

    public AutoMode(String name, Command command) {
      this.name = name;
      this.command = command;
    }
  }

  public static final String kAutoOptionsKey = "AutoStrategyOptions";
  public static final String kSelectedAutoModeKey = "AutoStrategy";
  public static final AutoMode kDefaultAutoMode = new AutoMode("All: Do Nothing", new Command() {
    @Override
    public Set<Subsystem> getRequirements() {
      return null;
    }
  });

  public final AutoMode[] mAutoModes;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    mAutoModes = new AutoMode[] {
      kDefaultAutoMode,
    };
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String selectedModeName = SmartDashboard.getString(kSelectedAutoModeKey, "NO SELECTED MODE!!!!");
    Logger.notice("Auto mode name " + selectedModeName);
    for (var mode : mAutoModes) {
      if (mode.name.equals(selectedModeName)) {
        return mode.command;
      }
    }

    Logger.error("AutoModeSelector failed to select auto mode: " + selectedModeName);
    return kDefaultAutoMode.command;
  }
}
