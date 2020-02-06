package com.spartronics4915.lib.subsystems;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

public class STOPCRASHESEXE
{

    public static boolean eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee = false;

    static
    {
        if (!eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee)
            nosegfaultplz();
        else
        {
            eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee = true;
        }
    }

    static void nosegfaultplz()
    {
        HAL.initialize(0xdad, 0xdad);
    }

    @Test
    public void justVibin()
    {}
}
