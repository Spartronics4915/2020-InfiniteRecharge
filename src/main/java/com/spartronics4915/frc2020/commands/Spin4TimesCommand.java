// to stop adding it:
// git reset src/main/java/com/spartronics4915/frc2020/commands/Spin4TimesCommand.java
package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.PanelRotator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Spin4TimesCommand extends CommandBase {

    private PanelRotator mPanelRotator;
    public double spins;
    public String firstColor;
    public String lastColor;
    public String currentColor;
    // You should only use one subsystem per command. If multiple are needed, use a CommandGroup.
    public Spin4TimesCommand(PanelRotator mSubsystem) {
      mPanelRotator = mSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      spins = 0;
      firstColor = mPanelRotator.getActualColor();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      currentColor = mPanelRotator.getActualColor();
      if(currentColor == firstColor && currentColor != lastColor)
        spins += .5;
      lastColor = mPanelRotator.getActualColor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if(spins == 4)
        return true;
      else
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end (boolean interrupted) {

    }
}