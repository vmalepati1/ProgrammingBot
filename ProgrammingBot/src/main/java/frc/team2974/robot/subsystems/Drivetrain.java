package frc.team2974.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.command.teleop.Drive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import java.io.IOException;

import static frc.team2974.robot.Config.SmartDashboardKeys.*;
import static frc.team2974.robot.Robot.drivetrain;
import static frc.team2974.robot.RobotMap.*;

public class Drivetrain extends Subsystem implements IPathfinderDrivetrain {

    private AHRS ahrs;

    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;
    private Notifier followerNotifier;

    public Drivetrain() {
        motorLeft.setInverted(true);

        encoderLeft.setReverseDirection(true);

        setDistancePerPulse();

        try {
            ahrs = new AHRS(SPI.Port.kMXP);
            System.out.println("Initialized NavX on SPI bus.");
        } catch (RuntimeException e) {
            e.printStackTrace();
        }

        followerNotifier = new Notifier(this::followPath);

        compressor.stop();
    }

    public AHRS getAhrs() {
        return ahrs;
    }

    public void shiftDown() {
        pneumaticsShifter.set(true);
    }

    public void shiftUp() {
        pneumaticsShifter.set(false);
    }

    public boolean isShiftDown() {
        return pneumaticsShifter.get();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive());
    }

    @Override
    public double getDistancePerPulse() {
        return 0.00038963112;
    }

    @Override
    public double getTrackWidth() {
        return 0.78;
    }

    @Override
    public int getTicksPerRevolution() {
        return 1024;
    }

    @Override
    public double getWheelDiameter() {
        return 0.127;
    }

    @Override
    public double getTimeStep() {
        return 0.02;
    }

    @Override
    public double getMaxVelocity() {
        return 2.634;
    }

    @Override
    public double getMaxAcceleration() {
        return 2;
    }

    @Override
    public double getMaxJerk() {
        return 60;
    }

    @Override
    public double getLeftFollowerKP() {
        return 0.3;
    }

    @Override
    public double getLeftFollowerKI() {
        return 0;
    }

    @Override
    public double getLeftFollowerKD() {
        return 0;
    }

    @Override
    public double getLeftFollowerKA() {
        return 0;
    }

    @Override
    public double getRightFollowerKP() {
        return 0.3;
    }

    @Override
    public double getRightFollowerKI() {
        return 0;
    }

    @Override
    public double getRightFollowerKD() {
        return 0;
    }

    @Override
    public double getRightFollowerKA() {
        return 0;
    }

    @Override
    public void setSpeeds(double leftPower, double rightPower) {
        motorRight.set(leftPower);
        motorLeft.set(rightPower);
    }

    @Override
    public void stopMotion() {
        setSpeeds(0, 0);
    }

    @Override
    public void resetEncoders() {
        encoderLeft.reset();
        encoderRight.reset();
    }

    @Override
    public void setDistancePerPulse() {
        encoderLeft.setDistancePerPulse(getDistancePerPulse());
        encoderRight.setDistancePerPulse(getDistancePerPulse());
    }

    @Override
    public void zeroYaw() {
        ahrs.zeroYaw();
    }

    @Override
    public int[] getEncoderPositions() {
        return new int[]{encoderLeft.get(), encoderRight.get()};
    }

    @Override
    public double[] getEncoderDistances() {
        return new double[]{encoderLeft.getDistance(), encoderRight.getDistance()};
    }

    @Override
    public void followPathSimple(Waypoint[] controlPoints) {
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH,
                getTimeStep(), getMaxVelocity(), getMaxAcceleration(), getMaxJerk());

        Trajectory trajectory = Pathfinder.generate(controlPoints, config);

        TankModifier modifier = new TankModifier(trajectory).modify(getTrackWidth());

        startFollowingNotifier(modifier.getLeftTrajectory(), modifier.getRightTrajectory());
    }

    @Override
    public void followPathCSV(String leftTrajectoryFilepath, String rightTrajectoryFilepath) {
        Trajectory leftTrajectory = null;
        Trajectory rightTrajectory = null;

        try {
            leftTrajectory = PathfinderFRC.getTrajectory(leftTrajectoryFilepath);
            rightTrajectory = PathfinderFRC.getTrajectory(rightTrajectoryFilepath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        startFollowingNotifier(leftTrajectory, rightTrajectory);
    }

    @Override
    public boolean isFollowingPath() {
        return !isDoneFollowingPath();
    }

    @Override
    public boolean isDoneFollowingPath() {
        return leftFollower.isFinished() || rightFollower.isFinished();
    }

    private void startFollowingNotifier(Trajectory leftTrajectory, Trajectory rightTrajectory) {
        // Stop any previously queued path
        followerNotifier.stop();

        leftFollower = new EncoderFollower(leftTrajectory);
        rightFollower = new EncoderFollower(rightTrajectory);

        leftFollower.configureEncoder(getEncoderPositions()[0], getTicksPerRevolution(), getWheelDiameter());
        leftFollower.configurePIDVA(getLeftFollowerKP(), getLeftFollowerKI(), getLeftFollowerKD(), 1 / getMaxVelocity(), getLeftFollowerKA());

        rightFollower.configureEncoder(getEncoderPositions()[1], getTicksPerRevolution(), getWheelDiameter());
        rightFollower.configurePIDVA(getRightFollowerKP(), getRightFollowerKI(), getRightFollowerKD(), 1 / getMaxVelocity(), getRightFollowerKA());

        // TODO: Maybe this is necessary?
        zeroYaw();

        followerNotifier.startPeriodic(leftTrajectory.get(0).dt);
    }

    private void followPath() {
        if (isDoneFollowingPath()) {
            followerNotifier.stop();
        } else {
            double leftSpeed = leftFollower.calculate(getEncoderPositions()[0]);
            double rightSpeed = rightFollower.calculate(getEncoderPositions()[0]);
            // TODO: Not sure if heading needs to be negated
            // TODO: Try fusedHeading() if this doesn't work
            double heading = drivetrain.getAhrs().getAngle();
            // TODO: Not sure about orientation either so desiredHeading may also need to be negated
            double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
            double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - heading);
            double turn = 0.8 * (-1.0 / 80.0) * headingDifference;
            drivetrain.setSpeeds(leftSpeed + turn, rightSpeed - turn);

            SmartDashboard.putNumber(PATHFINDER_HEADING, heading);
            SmartDashboard.putNumber(PATHFINDER_DESIRED_HEADING, desiredHeading);
            SmartDashboard.putNumber(PATHFINDER_HEADING_DIFFERENCE, headingDifference);
            SmartDashboard.putNumber(PATHFINDER_TURN, turn);
            SmartDashboard.putNumber(PATHFINDER_LEFT_MOTOR_SPEED, leftSpeed);
            SmartDashboard.putNumber(PATHFINDER_RIGHT_MOTOR_SPEED, rightSpeed);
        }
    }

}
