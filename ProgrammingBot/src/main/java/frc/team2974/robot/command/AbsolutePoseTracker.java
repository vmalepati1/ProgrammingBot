package frc.team2974.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.team2974.robot.Config.SmartDashboardKeys.*;
import static frc.team2974.robot.Robot.drivetrain;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;

public class AbsolutePoseTracker extends Command {

    private static AbsolutePoseTracker instance;

    private static double previousPoseMatrix[];
    private static double currentPoseMatrix[];

    private static double previousDistanceLeft;
    private static double previousDistanceRight;

    public AbsolutePoseTracker() {
        requires(drivetrain);
    }

    public static void setStartingPose(double[] startingPose) {
        previousPoseMatrix = startingPose;
        previousDistanceLeft = encoderLeft.getDistance();
        previousDistanceRight = encoderRight.getDistance();
    }

    public static AbsolutePoseTracker getInstance() {
        if (instance == null) {
            instance = new AbsolutePoseTracker();
        }
        return instance;
    }

    public static double[] getCurrentPoseMatrix() {
        return currentPoseMatrix;
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized absolute pose tracker.");

        previousPoseMatrix = new double[3];
        currentPoseMatrix = new double[3];

        previousDistanceLeft = 0;
        previousDistanceRight = 0;
    }

    @Override
    protected void execute() {
        double currentDistanceLeft = encoderLeft.getDistance();
        double currentDistanceRight = encoderRight.getDistance();
        double dLeft = currentDistanceLeft - previousDistanceLeft;
        double dRight = currentDistanceRight - previousDistanceRight;
        double dCenter = (dLeft + dRight) / 2;
        double theta = (dRight - dLeft) / drivetrain.getTrackWidth();

        currentPoseMatrix[0] = previousPoseMatrix[0] + dCenter * Math.cos(previousPoseMatrix[2]);
        currentPoseMatrix[1] = previousPoseMatrix[1] + dCenter * Math.sin(previousPoseMatrix[2]);
        currentPoseMatrix[2] = previousPoseMatrix[2] + theta;

        SmartDashboard.putNumber(ABSOLUTE_POSE_X, currentPoseMatrix[0]);
        SmartDashboard.putNumber(ABSOLUTE_POSE_Y, currentPoseMatrix[1]);
        SmartDashboard.putNumber(ABSOLUTE_POSE_ANGLE_DEGREES, Math.toDegrees(currentPoseMatrix[2]));

        previousPoseMatrix = currentPoseMatrix;
        previousDistanceLeft = currentDistanceLeft;
        previousDistanceRight = currentDistanceRight;
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        System.out.println("Ended absolute pose tracker.");
    }

    protected void interrupted() {
        end();
    }
}
