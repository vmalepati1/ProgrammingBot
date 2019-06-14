package frc.team2974.robot.command;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDBase;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import static frc.team2974.robot.Robot.drivetrain;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;
import static frc.team2974.robot.Config.PathfinderConstants.K_TICKS_PER_REVOLUTION;
import static frc.team2974.robot.Config.PathfinderConstants.K_WHEEL_DIAMETER;
import static frc.team2974.robot.Config.PathfinderConstants.K_MAX_VELOCITY;

import java.io.IOException;

public class PathfinderSplineTest extends Command {

    // Pathfinder variables
    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    private Notifier followerNotifier;

    public PathfinderSplineTest() {
        requires(drivetrain);
        System.out.println("Initialized Pathfinder spline test.");
    }

    @Override
    protected void initialize() {
        Trajectory leftTrajectory = null;
        Trajectory rightTrajectory = null;

        try {
            // PathWeaver supposedly swaps paths
            leftTrajectory = PathfinderFRC.getTrajectory("test.right");
            rightTrajectory = PathfinderFRC.getTrajectory("test.left");
        } catch (IOException e) {
            e.printStackTrace();
        }

        leftFollower = new EncoderFollower(leftTrajectory);
        rightFollower = new EncoderFollower(rightTrajectory);

        leftFollower.configureEncoder(encoderLeft.get(), K_TICKS_PER_REVOLUTION, K_WHEEL_DIAMETER);
        // You must tune the PID values on the following line!
        leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / K_MAX_VELOCITY, 0);

        rightFollower.configureEncoder(encoderRight.get(), K_TICKS_PER_REVOLUTION, K_WHEEL_DIAMETER);
        // You must tune the PID values on the following line!
        rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / K_MAX_VELOCITY, 0);

        // Maybe this is necessary?
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
            // Not sure if yaw needs to be negated
            double heading = -drivetrain.ahrs.getYaw();
            // Not sure about orientation either so desiredHeading may also need to be negated
            double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
            double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - heading);
            double turn =  0.8 * (-1.0/80.0) * headingDifference;
            drivetrain.setSpeeds(leftSpeed + turn, rightSpeed - turn);
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // Finish once path has been completed
        return leftFollower.isFinished() && rightFollower.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("Pathfinder spline test completed.");
        followerNotifier.stop();
        drivetrain.setSpeeds(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
    
}