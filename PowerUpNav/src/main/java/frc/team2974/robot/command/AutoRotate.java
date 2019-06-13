package frc.team2974.robot.command;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.Robot.drivetrain;

public class AutoRotate extends Command {

    double autoValue;

    public AutoRotate(double value) {
        requires(drivetrain);
        autoValue = value;

        System.out.println("AutoRotate initialized: value=" + autoValue);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("AutoRotate started: value=" + autoValue);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double leftPower = 0;
        double rightPower = 0;

        // rotate to absolute angle
        leftPower = -drivetrain.RotateTo((float) autoValue);
        rightPower = -leftPower;

        // set speeds
        drivetrain.setSpeeds(leftPower, rightPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        boolean isComplete = false;

        if (drivetrain.turnController.onTarget()) {
            if (drivetrain.turnController.isEnabled()) {
                drivetrain.turnController.disable();
            }
            isComplete = true;
        }

        return isComplete;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("AutoRotate completed: value=" + drivetrain.turnController.getSetpoint() + ".  Actual value: " + drivetrain.ahrs.getAngle());
        drivetrain.setSpeeds(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
