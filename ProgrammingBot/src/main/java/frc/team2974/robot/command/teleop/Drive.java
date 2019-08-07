package frc.team2974.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.OI.*;
import static frc.team2974.robot.Robot.drivetrain;

public class Drive extends Command {

    public Drive() {
        requires(drivetrain);
    }

    public double getLeftThrottle() {
        if (Math.abs(leftJoystick.getY()) < 0.3) {
            return 0;
        }
        return -leftJoystick.getY();
    }

    public double getRightThrottle() {
        if (Math.abs(rightJoystick.getY()) < 0.3) {
            return 0;
        }
        return -rightJoystick.getY();
    }

    private void tankDrive() {
        drivetrain.setSpeeds(getLeftThrottle(), getRightThrottle());
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized drive.");
    }

    @Override
    protected void execute() {
        tankDrive();

        if (shiftUp.get()) {
            drivetrain.shiftUp();
        }

        if (shiftDown.get()) {
            drivetrain.shiftDown();
        }
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        System.out.println("Ended drive.");

        drivetrain.setSpeeds(0, 0);
    }

    protected void interrupted() {
        end();
    }
}
