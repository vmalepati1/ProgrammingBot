package frc.team2974.robot;

/**
 * This holds many constant values for all parts of the robot. It increases efficiency and effectiveness of the code.
 */
public final class Config {

    private Config() {
    }

    public static final class Input {

        public static final int LEFT_JOYSTICK_PORT = 1;
        public static final int RIGHT_JOYSTICK_PORT = 0;
        public static final int GAMEPAD_PORT = 2;
        public static final int SHIFT_UP_PORT = 3;
        public static final int SHIFT_DOWN_PORT = 2;

        private Input() {
        }
    }

    public static final class Hardware {

        public static final int LEFT_MOTOR_CHANNEL = 1;
        public static final int LEFT_ENCODER_CHANNEL1 = 0;
        public static final int LEFT_ENCODER_CHANNEL2 = 1;

        public static final int RIGHT_MOTOR_CHANNEL = 0;
        public static final int RIGHT_ENCODER_CHANNEL1 = 2;
        public static final int RIGHT_ENCODER_CHANNEL2 = 3;

        public static final int SHIFTER_CHANNEL = 0;

        private Hardware() {
        }
    }

    public static final class SmartDashboardKeys {

        public static final String TESTING_AUTON_SELECT = "Testing/Auton Select";

        public static final String PATHFINDER_HEADING = "Pathfinder/Heading";
        public static final String PATHFINDER_DESIRED_HEADING = "Pathfinder/Desired Heading";
        public static final String PATHFINDER_HEADING_DIFFERENCE = "Pathfinder/Heading Difference";
        public static final String PATHFINDER_TURN = "Pathfinder/Turn";
        public static final String PATHFINDER_LEFT_MOTOR_SPEED = "Pathfinder/Left Motor Speed";
        public static final String PATHFINDER_RIGHT_MOTOR_SPEED = "Pathfinder/Right Motor Speed";

        public static final String ABSOLUTE_POSE_X = "Absolute Pose/X";
        public static final String ABSOLUTE_POSE_Y = "Absolute Pose/Y";
        public static final String ABSOLUTE_POSE_ANGLE_DEGREES = "Absolute Pose/Angle (Degrees)";

        public static final String VISION_STEER_K = "Vision/Steer K";
        public static final String VISION_DRIVE_K = "Vision/Drive K";
        public static final String VISION_DESIRED_TARGET_AREA = "Vision/Desired Target Area";
        public static final String VISION_MAX_DRIVE = "Vision/Max Drive";
        public static final String VISION_PERCENT_OF_TRAJECTORY = "Vision/Percent Of Trajectory";

    }

}
