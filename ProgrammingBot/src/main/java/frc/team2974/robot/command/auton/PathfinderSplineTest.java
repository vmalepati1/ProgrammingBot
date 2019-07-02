package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import java.io.IOException;

import static frc.team2974.robot.Config.PathfinderConstants.*;
import static frc.team2974.robot.Config.SmartDashboardKeys.PATHFINDER_LEFT_MOTOR_SPEED;
import static frc.team2974.robot.Config.SmartDashboardKeys.PATHFINDER_RIGHT_MOTOR_SPEED;
import static frc.team2974.robot.Robot.drivetrain;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;

public class PathfinderSplineTest extends Command {

    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    private Notifier followerNotifier;

    public PathfinderSplineTest() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized Pathfinder spline test.");

        Trajectory leftTrajectory = null;
        Trajectory rightTrajectory = null;

        try {
            // TODO: PathWeaver supposedly swaps paths
            // TODO: May have to change the filepaths when you put the CSV files on the robot filesystem
            leftTrajectory = PathfinderFRC.getTrajectory("test.right");
            rightTrajectory = PathfinderFRC.getTrajectory("test.left");
        } catch (IOException e) {
            e.printStackTrace();
        }

        leftFollower = new EncoderFollower(leftTrajectory);
        rightFollower = new EncoderFollower(rightTrajectory);

        leftFollower.configureEncoder(encoderLeft.get(), K_TICKS_PER_REVOLUTION, K_WHEEL_DIAMETER);
        // TODO: You must tune the PID values on the following line!
        leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / K_MAX_VELOCITY, 0);

        rightFollower.configureEncoder(encoderRight.get(), K_TICKS_PER_REVOLUTION, K_WHEEL_DIAMETER);
        // TODO: You must tune the PID values on the following line!
        rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / K_MAX_VELOCITY, 0);

        // TODO: Maybe this is necessary?
        drivetrain.ZeroYaw();

        followerNotifier = new Notifier(this::followPath);
        followerNotifier.startPeriodic(leftTrajectory.get(0).dt);
    }

    private void followPath() {
        if (leftFollower.isFinished() || rightFollower.isFinished()) {
            followerNotifier.stop();
        } else {
            double leftSpeed = leftFollower.calculate(encoderLeft.get());
            double rightSpeed = rightFollower.calculate(encoderRight.get());
            // TODO: Not sure if yaw needs to be negated
            double heading = -drivetrain.getAhrs().getYaw();
            // TODO: Not sure about orientation either so desiredHeading may also need to be negated
            double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
            double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - heading);
            double turn = 0.8 * (-1.0 / 80.0) * headingDifference;
            drivetrain.setSpeeds(leftSpeed + turn, rightSpeed - turn);

            SmartDashboard.putNumber(PATHFINDER_LEFT_MOTOR_SPEED, leftSpeed);
            SmartDashboard.putNumber(PATHFINDER_RIGHT_MOTOR_SPEED, rightSpeed);
        }
    }

    @Override
    protected void execute() {
    }

    protected boolean isFinished() {
        return leftFollower.isFinished() && rightFollower.isFinished();
    }

    protected void end() {
        System.out.println("Ended Pathfinder spline test.");

        followerNotifier.stop();
        drivetrain.setSpeeds(0, 0);
    }

    protected void interrupted() {
        end();
    }

}