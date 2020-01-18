// I'm bad at naming things. Please come up with a better name...
package com.spartronics4915.frc2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import 

import edu.wpi.first.wpilibj.DigitalInput;

public class PanelRotator extends SpartronicsSubsystem {
    private final CANSparkMax mSpinMotor;
    private final CANSparkMax mExtendMotor;
    private final DigitalInput mBeamSensorUp;
    private final DigitalInput mBeamSensorDown;
    private final ColorSensorV3 mColorSensorRight;
    private final ColorSensorV3 mColorSensorLeft;
    private final I2C.Port i2cPort = 

    public PanelRotator() {
        mBeamSensorUp = new DigitalInput(Constants.kBeamSensorUpID);
        mBeamSensorDown = new DigitalInput(Constants.kBeamSensorDownID);
        mSpinMotor = new CANSparkMax(Constants.kSpinMotorID, MotorType.kBrushless);
        mExtendMotor = new CANSparkMax(Constants.kExtendMotorID, MotorType.kBrushless);
        mColorSensorLeft = new ColorSensorV3(Constants.kColorSensorLeftID);
        mColorSensorRight = new ColorSensorV3(Constants.kColorSensorRightID);
    }

    public void raise() {
        mExtendMotor.set(0.5);
        if(!mBeamSensorUp.get())
            mExtendMotor.set(0);
    }
    public void lower() {
        mExtendMotor.set(-0.5);
        if(mBeamSensorDown.get())
            mExtendMotor.set(0);
    }

    public void spin(double speed) {
       
    }

    public String color() {
        return "red but that's a lie";
    }

    public void stop() {

    }
}