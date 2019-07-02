package frc.team2974.robot;

import edu.wpi.first.wpilibj.*;

import static frc.team2974.robot.Config.Hardware.*;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {

    public static final Talon motorLeft;
    public static final Talon motorRight;

    public static final Encoder encoderLeft;
    public static final Encoder encoderRight;

    public static final Compressor compressor;
    public static final Solenoid pneumaticsShifter;

    static {
        motorLeft = new Talon(LEFT_MOTOR_CHANNEL);
        motorRight = new Talon(RIGHT_MOTOR_CHANNEL);

        encoderRight = new Encoder(new DigitalInput(RIGHT_ENCODER_CHANNEL1),
                new DigitalInput(RIGHT_ENCODER_CHANNEL2));
        encoderLeft = new Encoder(new DigitalInput(LEFT_ENCODER_CHANNEL1),
                new DigitalInput(LEFT_ENCODER_CHANNEL2));

        compressor = new Compressor();
        pneumaticsShifter = new Solenoid(SHIFTER_CHANNEL);
    }

    private RobotMap() {
    }
}
