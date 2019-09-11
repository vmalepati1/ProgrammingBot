package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.Robot.drivetrain;

public class GeneralCurve extends Command {

    public GeneralCurve() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized drive to rocket.");

        drivetrain.followPathCSV("ToRedRocket.left", "ToRedRocket.right");
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return drivetrain.isDoneFollowingPath();
    }

    @Override
    protected void end() {
        System.out.println("Ended drive to rocket.");
    }

    @Override
    protected void interrupted() {
        System.out.println("Drive to rocket interrupted.");
        end();
    }

}
