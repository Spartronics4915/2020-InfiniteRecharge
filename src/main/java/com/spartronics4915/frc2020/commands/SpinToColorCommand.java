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
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        switch (gameData.charAt(0))
        {
          case 'B' :
            mPanelRotator.spin(0.1);
            break;
          case 'G' :
            //Green case code
            break;
          case 'R' :
            //Red case code
            break;
          case 'Y' :
            //Yellow case code
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
