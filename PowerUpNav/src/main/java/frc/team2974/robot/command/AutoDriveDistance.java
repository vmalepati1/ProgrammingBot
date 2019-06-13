package frc.team2974.robot.command;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.Robot.drivetrain;

public class AutoDriveDistance extends Command {

    double autoValue;

    public AutoDriveDistance(double value) {
        requires(drivetrain);
        autoValue = value;

        System.out.println("AutoDriveDistance initialized: value=" + autoValue);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("AutoDriveDistance started: value=" + autoValue);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double leftPower = 0;
        double rightPower = 0;

        // drive forward a specified distance at current heading
        drivetrain.SetTurnController(drivetrain.ahrs.getYaw());
        drivetrain.SetDistanceController(autoValue);
        leftPower = -drivetrain.driveToDistanceRate - drivetrain.rotateToAngleRate;
        ;
        rightPower = -drivetrain.driveToDistanceRate + drivetrain.rotateToAngleRate;

        // set speeds
        drivetrain.setSpeeds(leftPower, rightPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        boolean isComplete = false;

        if (drivetrain.turnController.onTarget() && drivetrain.distanceController.onTarget()) {
            if (drivetrain.turnController.isEnabled()) {
                drivetrain.turnController.disable();
            }
            if (drivetrain.distanceController.isEnabled()) {
                drivetrain.distanceController.disable();
            }
            isComplete = true;
        }

        return isComplete;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("AutoDriveDistance completed: value=" + autoValue);
        drivetrain.setSpeeds(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
