// I'm bad at naming things. Please come up with a better name...
package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DigitalInput;

public class PanelRotator extends SpartronicsSubsystem
{
    private final SpartronicsMotor mSpinMotor;
    private final SpartronicsMotor mExtendMotor;

    private final DigitalInput mBeamSensorUp;
    private final DigitalInput mBeamSensorDown;

    private final ColorSensorV3 mColorSensor;

    // TODO: These are essentially random numbers, with the max value based on the
    // images at
    // https://www.andymark.com/products/infinite-recharge-control-panel-stickr
    public int[] mMinimumRed = {200, 0, 0};
    public int[] mMaximumRed = {255, 30, 30};

    // TODO: These are bad and will work in a way that will make you lose, which
    // will be sad
    public int[] mMinimumGreen = {0, 200, 0};
    public int[] mMaximumGreen = {30, 255, 30};

    // TODO: These are bad and will work in a way that will make you lose, which
    // will be sad
    public int[] mMinimumBlue = {0, 200, 200};
    public int[] mMaximumBlue = {30, 255, 255};

    // TODO: These are bad and will work in a way that will make you lose, which
    // will be sad
    public int[] mMinimumYellow = {200, 200, 0};
    public int[] mMaximumYellow = {255, 255, 30};

    public String sensedColor;

    public int red;
    public int green;
    public int blue;

    public PanelRotator()
    {
        mBeamSensorUp = new DigitalInput(Constants.PanelRotator.kBeamSensorUpID);
        mBeamSensorDown = new DigitalInput(Constants.PanelRotator.kBeamSensorDownID);
        mSpinMotor = SpartronicsMax.makeMotor(Constants.PanelRotator.kSpinMotorID,
            SensorModel.fromMultiplier(1));
        mExtendMotor = SpartronicsSRX.makeMotor(Constants.PanelRotator.kExtendMotorID,
            SensorModel.fromMultiplier(1));
        // mSpinMotor = new CANSparkMax(Constants.PanelRotator.kSpinMotorID,
        // MotorType.kBrushless);
        // mExtendMotor = new TalonSRX(Constants.PanelRotator.kExtendMotorID);
        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    /** raises the arm holding the spinner at a set speed*/
    public void raise()
    {
        mExtendMotor.setDutyCycle(Constants.PanelRotator.kExtendMotorSpeed);
    }

    /** lowers the arm holding the spinner at a set speed*/
    public void lower()
    {
        mExtendMotor.setDutyCycle(-Constants.PanelRotator.kExtendMotorSpeed);
    }

    /** stops the extension motor */
    public void stopExtendMotor()
    {
        mExtendMotor.setDutyCycle(0);
    }

    /** gets the color (Red, Blue, Yellow, or Green) through game specific messages that the robot needs to spin to */
    public String getTargetColor()
    {
        return DriverStation.getInstance().getGameSpecificMessage();
    }

    /** finds what color the color sensor is seeing  (Red, Blue, Yellow, or Green); currently just a placeholder for output */
    public String getActualColor()
    {
        /*
        // TODO: convert to 0-255 for user convenience.
        red = mColorSensor.getRed();
        green = mColorSensor.getGreen();
        blue = mColorSensor.getBlue();
        sensedColor = "sensor is not working";
        if (mMinimumRed[0] <= red && red <= mMaximumRed[0])
        {
            if (mMinimumRed[1] <= green && green <= mMaximumRed[1])
            {
                if (mMinimumRed[2] <= blue && blue <= mMaximumRed[2])
                {
                    sensedColor = "Red";
                }
            }
        }
        if (mMinimumBlue[0] <= red && red <= mMaximumBlue[0])
        {
            if (mMinimumBlue[1] <= green && green <= mMaximumBlue[1])
            {
                if (mMinimumBlue[2] <= blue && blue <= mMaximumBlue[2])
                {
                    sensedColor = "Blue";
                }
            }
        }
        if (mMinimumYellow[0] <= red && red <= mMaximumYellow[0])
        {
            if (mMinimumYellow[1] <= green && green <= mMaximumYellow[1])
            {
                if (mMinimumYellow[2] <= blue && blue <= mMaximumYellow[2])
                {
                    sensedColor = "Yellow";
                }
            }
        }
        if (mMinimumGreen[0] <= red && red <= mMaximumGreen[0])
        {
            if (mMinimumGreen[1] <= green && green <= mMaximumGreen[1])
            {
                if (mMinimumGreen[2] <= blue && blue <= mMaximumGreen[2])
                {
                    sensedColor = "Green";
                }
            }
        }
        System.out.println(sensedColor);
        return sensedColor;
        */
        return "method not complete";
    }

    /** sees if the bottom beam sensor is triggered */
    public boolean getBeamSensorDown()
    {
        // TODO: maybe backwards
        return mBeamSensorDown.get();
    }

    /** sees if the top beam sensor is triggered */
    public boolean getBeamSensorUp()
    {
        return mBeamSensorUp.get(); // TODO: maybe backwards
    }

    /** spins the wheel to move the control panel */
    public void spin()
    {
        mSpinMotor.setDutyCycle(Constants.PanelRotator.kSpinMotorSpeed);
    }

    /** get the number of times that the spinning */
    public double getRotations()
    {
        return -1; // TODO: not complete
    }

    /** stops the wheel */
    public void stopSpin()
    {
        mSpinMotor.setDutyCycle(0);
    }

    /** stops the two motors */
    public void stop()
    {
        mSpinMotor.setDutyCycle(0);
        mExtendMotor.setDutyCycle(0);
    }
}
