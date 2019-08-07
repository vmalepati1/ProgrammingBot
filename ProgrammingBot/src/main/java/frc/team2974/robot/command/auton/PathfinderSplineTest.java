package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Waypoint;

import static frc.team2974.robot.Robot.drivetrain;

public class PathfinderSplineTest extends Command {

    public PathfinderSplineTest() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized Pathfinder spline test.");

        // Drive forward exactly 4 meters straight

        drivetrain.followPathCSV("curved.left", "curved.right");
        //drivetrain.followPathSimple(new Waypoint[]{new Waypoint(0, 0, 0), new Waypoint(0, 4, 0)});
    }

    @Override
    protected void execute() {
    }

    protected boolean isFinished() {
        return drivetrain.isDoneFollowingPath();
    }

    protected void end() {
        System.out.println("Ended Pathfinder spline test.");

        drivetrain.stopMotion();
    }

    protected void interrupted() {
        end();
    }

}