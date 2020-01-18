// I'm bad at naming things. Please come up with a better name...
package com.spartronics4915.frc2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class PanelRotator extends SpartronicsSubsystem {
    private final CANSparkMax mPanelMotor;
    private final DigitalInput mBeamSensor;

    public PanelRotator() {
        mBeamSensor = new DigitalInput(Constants.kBeamSensorID);
        mPanelMotor = new CANSparkMax(Constants.kPanelMotorID, MotorType.kBrushless);
    }

    public void rotate(double number) {
        if(mBeamSensor.get())
            mPanelMotor.set(0);
    }

    public String color() {
        return "red but that's a lie";
    }

    public void stop() {

    }
}