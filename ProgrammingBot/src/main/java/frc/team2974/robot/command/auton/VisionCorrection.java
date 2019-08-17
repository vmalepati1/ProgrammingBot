package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;

import static frc.team2974.robot.Robot.drivetrain;

public class VisionCorrection extends Command {

    public VisionCorrection() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized vision correction.");
    }

    @Override
    protected void execute() {
        drivetrain.updateLimelightTracking();

        if (drivetrain.isLimelightHasValidTarget()) {
            drivetrain.setArcadeSpeeds(drivetrain.getLimelightDriveCommand(), drivetrain.getLimelightSteerCommand());
        } else {
            drivetrain.stopMotion();
        }
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
