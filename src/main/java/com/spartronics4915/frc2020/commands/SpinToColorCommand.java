package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.PanelRotator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinToColorCommand extends CommandBase {

    private PanelRotator mPanelRotator;
    // You should only use one subsystem per command. If multiple are needed, use a CommandGroup.
    public SpinToColorCommand(PanelRotator mSubsystem) {
      mPanelRotator = mSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        String targetColor = mPanelRotator.getTargetColor();
        targetColor = DriverStation.getInstance().getGameSpecificMessage();
        switch (targetColor.charAt(0))
        {
          case 'B' :
            if(mPanelRotator.getActualColor() != "Blue") {
              mPanelRotator.spin(0.1);}
            break;
          case 'G' :
          if(mPanelRotator.getActualColor() != "Green") {
            mPanelRotator.spin(0.1);}
            break;
          case 'R' :
          if(mPanelRotator.getActualColor() != "Red") {
            mPanelRotator.spin(0.1);}
            break;
          case 'Y' :
          if(mPanelRotator.getActualColor() != "Yellow") {
            mPanelRotator.spin(0.1);}
            break;
          default :
            //This is corrupt data
            break;
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end (boolean interrupted) {

    }
}
