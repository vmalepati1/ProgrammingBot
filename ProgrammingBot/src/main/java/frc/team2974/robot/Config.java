package frc.team2974.robot;

/**
 * This holds many constant values for all parts of the robot. It increases efficiency and effectiveness of the code.
 */
public final class Config {

    public static final class Camera {

        public static final int DRIVER_PIPELINE = 3;
        public static final int AUTO_ALIGN_PIPELINE = 2;

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

        public static final String AUTON_COMMAND_GROUP = "Auton/Command Group";

        public static final String PATHFINDER_HEADING = "Pathfinder/Heading";
        public static final String PATHFINDER_DESIRED_HEADING = "Pathfinder/Desired Heading";
        public static final String PATHFINDER_HEADING_DIFFERENCE = "Pathfinder/Heading Difference";
        public static final String PATHFINDER_TURN = "Pathfinder/Turn";
        public static final String PATHFINDER_LEFT_MOTOR_SPEED = "Pathfinder/Left Motor Speed";
        public static final String PATHFINDER_RIGHT_MOTOR_SPEED = "Pathfinder/Right Motor Speed";

        public static final String PATHFINDER_LEFT_FOLLOWER_KP = "Pathfinder/Left Follower KP";
        public static final String PATHFINDER_LEFT_FOLLOWER_KD = "Pathfinder/Left Follower KP";
        public static final String PATHFINDER_RIGHT_FOLLOWER_KP = "Pathfinder/Right Follower KP";
        public static final String PATHFINDER_RIGHT_FOLLOWER_KD = "Pathfinder/Right Follower KD";

        public static final String ABSOLUTE_POSE_X = "Absolute Pose/X";
        public static final String ABSOLUTE_POSE_Y = "Absolute Pose/Y";
        public static final String ABSOLUTE_POSE_ANGLE_DEGREES = "Absolute Pose/Angle (Degrees)";

        public static final String VISION_STEER_K = "Vision/Steer K";
        public static final String VISION_DRIVE_K = "Vision/Drive K";
        public static final String VISION_DESIRED_TARGET_AREA = "Vision/Desired Target Area";
        public static final String VISION_MAX_DRIVE = "Vision/Max Drive";
        public static final String VISION_ALIGNED_X_PLUS_MINUS = "Vision/Aligned X Plus or Minus";
        public static final String VISION_ALIGNED_Z_PLUS_MINUS = "Vision/Aligned Z Plus or Minus";
    }

    public static final class PathfinderConstantsDefaults {

        public static final double PATHFINDER_LEFT_FOLLOWER_KP_DEFAULT = 0.0015;
        public static final double PATHFINDER_LEFT_FOLLOWER_KD_DEFAULT = 0;
        public static final double PATHFINDER_RIGHT_FOLLOWER_KP_DEFAULT = 0.00001;
        public static final double PATHFINDER_RIGHT_FOLLOWER_KD_DEFAULT = 0;

    }

    public static final class VisionConstantsDefaults {

        public static final double VISION_STEER_K_DEFAULT = 0.015;
        public static final double VISION_DRIVE_K_DEFAULT = 0.1;
        public static final double VISION_DESIRED_TARGET_AREA_DEFAULT = 13;
        public static final double VISION_MAX_DRIVE_DEFAULT = 0.8;

        public static final double VISION_ALIGNED_X_PLUS_MINUS_DEFAULT = 6;
        public static final double VISION_ALIGNED_Z_PLUS_MINUS_DEFAULT = 36;

    }

}
