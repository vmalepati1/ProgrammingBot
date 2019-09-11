package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.Robot.drivetrain;

public class WaitForTx extends Command {

    public WaitForTx() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized wait for tx.");
    }

    @Override
    protected void execute() {
        System.out.println(drivetrain.getTv());
    }

    @Override
    protected boolean isFinished() {
        return drivetrain.getTv() >= 1;
    }

    @Override
    protected void end() {
        System.out.println("Ended wait for tx.");
    }

    @Override
    protected void interrupted() {
        System.out.println("Wait for tx interrupted.");
        end();
    }

}