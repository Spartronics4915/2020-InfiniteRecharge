package com.spartronics4915.frc2020.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ProximitySensorCommand extends CommandBase
{
    private DigitalInput mProximitySensor;
    public void ProximitySensorCommand() 
    {  
        mProximitySensor = new DigitalInput(2);
        
    }
}
