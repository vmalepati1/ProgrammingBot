package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.io.IOException;

import static frc.team2974.robot.Robot.drivetrain;

public class VisionLine extends Command {

    public VisionLine() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized vision correction.");

        Trajectory leftTrajectory = null;
        Trajectory rightTrajectory = null;

        try {
            leftTrajectory = PathfinderFRC.getTrajectory("VisionLine.left");
            rightTrajectory = PathfinderFRC.getTrajectory("VisionLine.right");
        } catch (IOException e) {
            e.printStackTrace();
        }

        for (int i = 0; i < rightTrajectory.segments.length; i++) {
            rightTrajectory.segments[i].heading = drivetrain.getTx();
        }

        drivetrain.followTrajectory(leftTrajectory, rightTrajectory);
    }

    @Override
    protected void execute() {

    }

    @Override
    protected boolean isFinished() {
        // return drivetrain.isAligned();
        return false;
    }

    @Override
    protected void end() {
        System.out.println("Ended vision correction.");

        drivetrain.stopMotion();
    }

    @Override
    protected void interrupted() {
        System.out.println("Vision correction interrupted.");
        end();
    }

}
