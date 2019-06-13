package frc.team2974.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.Robot.drivetrain;

//import frc.team2974.robot.OI;

/**
 *
 */
public class Stop extends Command {

    public Stop() {
        requires(drivetrain);
    }

    protected void initialize() {
        System.out.println("Initialize");
        drivetrain.setSpeeds(0, 0);

    }

    protected void execute() {
        System.out.println("Execute");
        drivetrain.setSpeeds(0, 0);
    }

    protected boolean isFinished() {
        return false;
    }
}
