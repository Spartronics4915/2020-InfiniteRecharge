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

    //TODO: These are essentially random numbers, with the max value based on the images at
    // https://www.andymark.com/products/infinite-recharge-control-panel-stickr
    public int[] mMinimumRed = {200, 0, 0};
    public int[] mMaximumRed = {255, 30, 30};

    //TODO: These are bad and will work in a way that will make you lose, which will be sad
    public int[] mMinimumGreen = {0, 200, 0};
    public int[] mMaximumGreen = {30, 255, 30};

    //TODO: These are bad and will work in a way that will make you lose, which will be sad
    public int[] mMinimumBlue = {0, 200, 200};
    public int[] mMaximumBlue = {30, 255, 255};

    //TODO: These are bad and will work in a way that will make you lose, which will be sad
    public int[] mMinimumYellow = {200, 200, 0};
    public int[] mMaximumYellow = {255, 255, 30};

    public String sensedColor;
    
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

    public String getClosestColor() {
        //TODO: convert to 0-255 for user convenience.
        red =  mColorSensor.getRed();
        green =  mColorSensor.getGreen();
        blue =  mColorSensor.getBlue();
        sensedColor = "sensor is not working";
        if(mMinimumRed[0] <= red && red <= mMaximumRed[0]){
            if(mMinimumRed[1] <= green && green <= mMaximumRed[1]){
                if(mMinimumRed[2] <= blue && blue <= mMaximumRed[2]){
                    sensedColor = "Red";
                }
            }
        }
        if(mMinimumBlue[0] <= red && red <= mMaximumBlue[0]){
            if(mMinimumBlue[1] <= green && green <= mMaximumBlue[1]){
                if(mMinimumBlue[2] <= blue && blue <= mMaximumBlue[2]){
                    sensedColor = "Blue";
                }
            }
        }
        if(mMinimumYellow[0] <= red && red <= mMaximumYellow[0]){
            if(mMinimumYellow[1] <= green && green <= mMaximumYellow[1]){
                if(mMinimumYellow[2] <= blue && blue <= mMaximumYellow[2]){
                    sensedColor = "Yellow";
                }
            }
        }
        if(mMinimumGreen[0] <= red && red <= mMaximumGreen[0]){
            if(mMinimumGreen[1] <= green && green <= mMaximumGreen[1]){
                if(mMinimumGreen[2] <= blue && blue <= mMaximumGreen[2]){
                    sensedColor = "Green";
                }
            }
        }
        return sensedColor;
    }

    public void stop() {
        mSpinMotor.set(0);
        mExtendMotor.set(0);
    }
}