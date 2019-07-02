package frc.team2974.robot;

/**
 * This holds many constant values for all parts of the robot. It increases efficiency and effectiveness of the code.
 */
public final class Config {

    private Config() {
    }

    public static final class Input {

        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
        public static final int GAMEPAD_PORT = 2;
        public static final int SHIFT_UP_PORT = 3;
        public static final int SHIFT_DOWN_PORT = 2;

        private Input() {
        }
    }

    public static final class Hardware {

        public static final int LEFT_MOTOR_CHANNEL = 0;
        public static final int LEFT_ENCODER_CHANNEL1 = 2;
        public static final int LEFT_ENCODER_CHANNEL2 = 3;

        public static final int RIGHT_MOTOR_CHANNEL = 1;
        public static final int RIGHT_ENCODER_CHANNEL1 = 0;
        public static final int RIGHT_ENCODER_CHANNEL2 = 1;

        public static final int SHIFTER_CHANNEL = 0;

        private Hardware() {
        }
    }

    public static final class SmartDashboardKeys {

        public static final String TESTING_AUTON_SELECT = "Testing/Auton Select";

        public static final String PATHFINDER_LEFT_MOTOR_SPEED = "Pathfinder/Left Motor Speed";
        public static final String PATHFINDER_RIGHT_MOTOR_SPEED = "Pathfinder/Right Motor Speed";

        public static final String ABSOLUTE_POSE_X = "Absolute Pose/X";
        public static final String ABSOLUTE_POSE_Y = "Absolute Pose/Y";
        public static final String ABSOLUTE_POSE_ANGLE_DEGREES = "Absolute Pose/Angle (Degrees)";

    }

    public static final class PathfinderConstants {
        public static final int K_TICKS_PER_REVOLUTION = 1024;
        public static final double K_WHEEL_DIAMETER = 4.0 / 12.0;
        public static final double K_MAX_VELOCITY = 10;

        private PathfinderConstants() {
        }
    }

    public static final class RobotConstants {

        public static final double DISTANCE_PER_PULSE = 0.000409;
        public static final double DISTANCE_BETWEEN_WHEELS = 0.78;

    }

}
