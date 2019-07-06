package frc.team2974.robot.subsystems;

import jaci.pathfinder.Waypoint;

public interface IPathfinderDrivetrain {

    /* Note: All distance measurements should use the same unit. */

    /**
     * @return distance per PWM duty cycle
     */
    double getDistancePerPulse();

    /**
     * @return distance between wheels
     */
    double getTrackWidth();

    /**
     * @return encoder ticks per revolution
     */
    double getTicksPerRevolution();

    /**
     * @return wheel diameter
     */
    double getWheelDiameter();

    /**
     * @return maximum velocity in units/sec
     */
    double getMaxVelocity();

    /**
     * @return proportional gain constant for left drivetrain speed PID
     */
    double getLeftFollowerKP();

    /**
     * @return integral gain constant for left drivetrain speed PID (set this
     * to zero as only proportional and (maybe) derivative constants are needed for motion profiling)
     */
    double getLeftFollowerKI();

    /**
     * @return derivative gain constant for left drivetrain speed PID (at first set this to zero and
     * adjust the proportional constant)
     */
    double getLeftFollowerKD();

    /**
     * @return acceleration gain for left drivetrain speed PID (adjust to affect the speed change in
     * within a short time step)
     */
    double getLeftFollowerKA();

    /**
     * @return proportional gain constant for right drivetrain speed PID
     */
    double getRightFollowerKP();

    /**
     * @returni ntegral gain constant for right drivetrain speed PID (set this
     * to zero as only proportional and (maybe) derivative constants are needed for motion profiling)
     */
    double getRightFollowerKI();

    /**
     * @return derivative gain constant for right drivetrain speed PID (at first set this to zero and
     * adjust the proportional constant)
     */
    double getRightFollowerKD();

    /**
     * @return acceleration gain for right drivetrain speed PID (adjust to affect the speed change in
     * within a short time step)
     */
    double getRightFollowerKA();

    /**
     * @param leftSpeed: left drivetrain speed
     * @param rightSpeed: right drivetrain speed
     */
    void setSpeeds(double leftSpeed, double rightSpeed);

    /**
     * Set drivetrain speeds to zero.
     */
    void stopMotion();

    /**
     * Reset drivetrain encoders.
     */
    void resetEncoders();

    /**
     * Zero the yaw on the rotational tracking device.
     */
    void zeroYaw();

    /**
     * @return drivetrain encoder positions
     */
    double[] getEncoderPositions();

    /**
     * @param controlPoints: control points of Hermite cubic spline to follow
     */
    void followPathSimple(Waypoint[] controlPoints);

    /**
     * @param leftTrajectoryFilepath: filepath to left trajectory file with file extension ".left" (ex: "straight.left")
     * @param rightTrajectoryFilepath: filepath to right trajectory file with file extension ".right" (ex: "straight.right")
     */
    void followPathCSV(String leftTrajectoryFilepath, String rightTrajectoryFilepath);

}
