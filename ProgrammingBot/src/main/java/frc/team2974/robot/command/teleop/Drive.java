package frc.team2974.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.util.EnhancedBoolean;

import static frc.team2974.robot.OI.*;
import static frc.team2974.robot.Robot.drivetrain;

public class Drive extends Command {

    private final EnhancedBoolean rightTriggerPress = new EnhancedBoolean();
    private boolean isAligning = false;

    public Drive() {
        requires(drivetrain);
    }

    private double getLeftThrottle() {
        if (Math.abs(leftJoystick.getY()) < 0.3) {
            return 0;
        }
        return -leftJoystick.getY();
    }

    private double getRightThrottle() {
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
        rightTriggerPress.set(rightJoystick.getTrigger());

        if (rightTriggerPress.isRisingEdge()) {
            drivetrain.setLimelightAutoAlignPipeline();
        } else if (rightTriggerPress.isFallingEdge()) {
            drivetrain.setLimelightDriverPipeline();
        }

        if (rightTriggerPress.get()) {
            if (drivetrain.isLimelightHasValidTarget()) {
                drivetrain.setArcadeSpeeds(drivetrain.getLimelightDriveCommand(), drivetrain.getLimelightSteerCommand());
                isAligning = true;
            } else {
                isAligning = false;
            }
        } else if (rightTriggerPress.isFallingEdge()) {
            isAligning = false;
        }

        if (!isAligning || !drivetrain.isLimelightHasValidTarget()) {
            tankDrive();
        }

        if (shiftUp.get()) {
            drivetrain.shiftUp();
        }

        if (shiftDown.get()) {
            drivetrain.shiftDown();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        System.out.println("Ended drive.");

        drivetrain.stopMotion();
    }

    @Override
    protected void interrupted() {
        end();
    }

}
