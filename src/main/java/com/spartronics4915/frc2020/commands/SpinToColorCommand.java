package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.PanelRotator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinToColorCommand extends CommandBase {

  private PanelRotator mPanelRotator;
  private String targetColor;

  // You should only use one subsystem per command. If multiple are needed, use a
  // CommandGroup.
  public SpinToColorCommand(PanelRotator mSubsystem) {
    mPanelRotator = mSubsystem;
    addRequirements(mPanelRotator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetColor = mPanelRotator.getTargetColor();
    if (mPanelRotator.getActualColor() != targetColor && targetColor.length() > 0)
      mPanelRotator.spin();
    else
      targetColor = "error";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mPanelRotator.getActualColor() == DriverStation.getInstance().getGameSpecificMessage()) {
      return true;
    } else if (targetColor == "error") {
      System.out.println("error: no target color given. SpinToColorCommand terminated.");
      return true;
    } else {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mPanelRotator.stopSpin();
  }
}