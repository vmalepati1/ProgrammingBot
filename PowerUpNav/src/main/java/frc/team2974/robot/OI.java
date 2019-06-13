package frc.team2974.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team2974.robot.command.AutoCommandGroup;

import static frc.team2974.robot.Config.Input.GAMEPAD_PORT;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public final class OI {

    public static final Gamepad gamepad;

    static {
        gamepad = new Gamepad(GAMEPAD_PORT);
    }

    Button button9 = new JoystickButton(gamepad, 9);
    Button button10 = new JoystickButton(gamepad, 10);
  
  /*Button button1 = new JoystickButton(gamepad, 1),
  button2 = new JoystickButton(gamepad, 2),
  button3 = new JoystickButton(gamepad, 3), 
  button9 = new JoystickButton(gamepad, 4),
  buttonLeftTrigger = new JoystickButton(gamepad, 7),
  buttonRightTrigger = new JoystickButton(gamepad, 8);*/

    public OI() {
        button9.whenPressed(new AutoCommandGroup(1));
        button10.whenPressed(new AutoCommandGroup(2));
    }
}
