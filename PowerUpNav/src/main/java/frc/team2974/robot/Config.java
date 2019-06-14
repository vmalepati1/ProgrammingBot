package frc.team2974.robot;

/**
 * This holds many constant values for all parts of the robot. It increases efficiency and effectiveness of the code.
 */
public final class Config {

    private Config() {
    }

    public enum Robot {
        PRACTICE(0.7800, 0.000409, 775, false, false);
        //COMPETITION(0.7800, 0.0002045, 386, true, true);

        private final double robotWidth;
        private final double distancePerPulse;
        private final int inchesToNativeUnits;
        private final boolean isReversed;
        private final boolean sensorPhase;

        Robot(double robotWidth, double distancePerPulse, int inchesToNativeUnits, boolean isReversed,
              boolean SensorPhase) {

            this.robotWidth = robotWidth;
            this.distancePerPulse = distancePerPulse;
            this.inchesToNativeUnits = inchesToNativeUnits;
            this.isReversed = isReversed;
            sensorPhase = SensorPhase;
        }

        public int getInchesToNativeUnits() {
            return inchesToNativeUnits;
        }

        public double getRobotWidth() {
            return robotWidth;
        }

        public double getDistancePerPulse() {
            return distancePerPulse;
        }

        public boolean getSensorPhase() {
            return sensorPhase;
        }

        public boolean isReversed() {
            return isReversed;
        }
    }

    public static final class Hardware {

        public static final int LEFT_MOTOR_CHANNEL = 0; // pwm
        public static final int LEFT_ENCODER_CHANNEL1 = 2; // for first digital input
        public static final int LEFT_ENCODER_CHANNEL2 = 3; // for second digital input

        public static final int RIGHT_MOTOR_CHANNEL = 1; // pwm
        public static final int RIGHT_ENCODER_CHANNEL1 = 0; // digital
        public static final int RIGHT_ENCODER_CHANNEL2 = 1; // digital

        public static final int SHIFTER_CHANNEL = 0; // pcm

        public static final int ROBOT_IDENTIFIER_CHANNEL = 9;

        private Hardware() {
        }
    }

    public static final class Input {

        public static final int GAMEPAD_PORT = 0;

        private Input() {
        }
    }

    public static final class Path {

        public static final double CROSS_BASELINE_Y = 4.2748092;

        public static final double VELOCITY_MAX = 4; /* 3.075548163 TESTED MAX VELOCITY*/ //3 m/s
        public static final double ACCELERATION_MAX = 3.5; //2 m/s^2

    }

    public static final class MotionConstants {

        public static final double KV = 0.194350; // 0.267
        public static final double KAcc = 0.067555; //0
        public static final double KK = 0.193466; // 0

        public static final double KS = 2;//1
        public static final double KAng = 1;//1
        public static final double KL = 2; //2

        public static final double IL = 0.00; // 0.01
        public static final double IAng = 0.005; // 0.01

        private MotionConstants() {
        }
    }

    public static final class PathfinderConstants {
        public static final int K_TICKS_PER_REVOLUTION = 1024;
        public static final double K_WHEEL_DIAMETER = 4.0 / 12.0;
        public static final double K_MAX_VELOCITY = 10;

        private PathfinderConstants() {
        }
    }
}
