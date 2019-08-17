package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.Robot.drivetrain;

public class StraightTuning extends Command {

    public StraightTuning() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized straight tuning.");

        // Drive forward exactly 4 meters straight

        drivetrain.followPathCSV("Straight4Meter.left", "Straight4Meter.right");
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
        System.out.println("Ended straight tuning.");

        drivetrain.stopMotion();
    }

    @Override
    protected void interrupted() {
        System.out.println("Straight tuning interrupted.");
        end();
    }

}