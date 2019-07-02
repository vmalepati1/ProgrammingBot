package frc.team2974.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import static frc.team2974.robot.Config.Input.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public final class OI {

    public static final Joystick leftJoystick;
    public static final Joystick rightJoystick;
    public static final Gamepad gamepad;

    public static final Button shiftUp;
    public static final Button shiftDown;

    static {
        leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
        rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
        gamepad = new Gamepad(GAMEPAD_PORT);

        shiftUp = new JoystickButton(leftJoystick, SHIFT_UP_PORT);
        shiftDown = new JoystickButton(leftJoystick, SHIFT_DOWN_PORT);
    }

    public OI() {

    }
}
