// I'm bad at naming things. Please come up with a better name...
package com.spartronics4915.frc2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DigitalInput;

public class PanelRotator extends SpartronicsSubsystem {
    private final CANSparkMax mSpinMotor;
    private final CANSparkMax mExtendMotor;
    
    private final DigitalInput mBeamSensorUp;
    private final DigitalInput mBeamSensorDown;
    
    private final ColorSensorV3  mColorSensor;

    private int[] mMinimumRed = {255, 0, 0};
    private int[] mMaximumRed = {255, 0, 0};  
    
    public int red;
    public int green;
    public int blue;

    public PanelRotator() {
        mBeamSensorUp = new DigitalInput(Constants.kPanelRotatorBeamSensorUpID);
        mBeamSensorDown = new DigitalInput(Constants.kPanelRotatorBeamSensorDownID);
        mSpinMotor = new CANSparkMax(Constants.kPanelRotatorSpinMotorID, MotorType.kBrushless);
        mExtendMotor = new CANSparkMax(Constants.kPanelRotatorExtendMotorID, MotorType.kBrushless);
        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    public void raise() {
        mExtendMotor.set(Constants.kPanelRotatorExtendMotorSpeed);
        if(!mBeamSensorUp.get())
            mExtendMotor.set(0);
    }

    public void lower() {
        mExtendMotor.set(-Constants.kPanelRotatorExtendMotorSpeed);
        if(mBeamSensorDown.get())
            mExtendMotor.set(0);
    }

    public String getTargetColor() {
        String color = DriverStation.getInstance().getGameSpecificMessage();
        return color;
    }

    /*public String getClosestColor() {
        red =  mColorSensor.getRed();
        green =  mColorSensor.getGreen();
        blue =  mColorSensor.getBlue();
        
    }*/

    public void stop() {

    }
}