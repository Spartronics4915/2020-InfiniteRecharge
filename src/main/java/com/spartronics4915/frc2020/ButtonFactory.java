package com.spartronics4915.frc2020;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

public class ButtonFactory
{
    /**
     * create a joystick button if the current machine configuration
     * supports it. We dance this little jig to reduce console spewage.
     * @param js
     * @param buttonid
     * @return either a Joystick button or an InvalidButton
     */
    static Button Create(Joystick js, int buttonid)
    {
        int port = -1;
        if(js != null)
            port = js.getPort();

        int newId = Constants.OI.getRemappedButton(port, buttonid);
        if(newId == -1)
            return new InvalidButton(0, buttonid);
        else
            return new JoystickButton(js, buttonid);
    }

    /* we don't extend JoystickButton because the super is always
     * called and this is the source of some spewage when missing.
     */
    private static class InvalidButton extends Button
    {
        public InvalidButton(int port, int button) 
        {
        }

        /* overriding core Trigger methods gets us most of the way since Button
         * is a thin vaneer atop Trigger
         */
        @Override
        public Trigger whenActive(final Command toRun, boolean inter)
        {
            // no-op
            return this;
        }

        @Override
        public Trigger whileActiveContinuous(final Command toRun, boolean inter)
        {
            // no-op
            return this;
        }

        @Override
        public Trigger whileActiveOnce(final Command toRun, boolean inter)
        {
            // no-op
            return this;
        }

        @Override
        public Button whenInactive(final Command toRun, boolean intr)
        {
            // no-op
            return this;
        }

        @Override
        public Button toggleWhenActive(final Command toRun, boolean intr)
        {
            // no-op
            return this;
        }

        @Override
        public Button cancelWhenActive(final Command toRun)
        {
            // no-op
            return this;
        }

    }
}