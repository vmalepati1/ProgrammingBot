package frc.team2974.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team2974.robot.command.teleop.Drive;
import jaci.pathfinder.Waypoint;

import static frc.team2974.robot.Config.RobotConstants.DISTANCE_PER_PULSE;
import static frc.team2974.robot.RobotMap.*;

public class Drivetrain extends Subsystem implements IPathfinderDrivetrain {

    private AHRS ahrs;

    public Drivetrain() {
        motorLeft.setInverted(true);

        encoderLeft.setReverseDirection(true);

        encoderLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
        encoderRight.setDistancePerPulse(DISTANCE_PER_PULSE);

        try {
            ahrs = new AHRS(SPI.Port.kMXP);
            System.out.println("Initialized NavX on SPI bus.");
        } catch (RuntimeException e) {
            e.printStackTrace();
        }

        compressor.stop();
    }

    public AHRS getAhrs() {
        return ahrs;
    }

    public void shiftDown() {
        pneumaticsShifter.set(true);
    }

    public void shiftUp() {
        pneumaticsShifter.set(false);
    }

    public boolean isShiftDown() {
        return pneumaticsShifter.get();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive());
    }

    @Override
    public double getDistancePerPulse() {
        return 0;
    }

    @Override
    public double getTrackWidth() {
        return 0;
    }

    @Override
    public double getTicksPerRevolution() {
        return 0;
    }

    @Override
    public double getWheelDiameter() {
        return 0;
    }

    @Override
    public double getMaxVelocity() {
        return 0;
    }

    @Override
    public double getLeftFollowerKP() {
        return 0;
    }

    @Override
    public double getLeftFollowerKI() {
        return 0;
    }

    @Override
    public double getLeftFollowerKD() {
        return 0;
    }

    @Override
    public double getLeftFollowerKA() {
        return 0;
    }

    @Override
    public double getRightFollowerKP() {
        return 0;
    }

    @Override
    public double getRightFollowerKI() {
        return 0;
    }

    @Override
    public double getRightFollowerKD() {
        return 0;
    }

    @Override
    public double getRightFollowerKA() {
        return 0;
    }

    @Override
    public void setSpeeds(double leftPower, double rightPower) {
        motorRight.set(leftPower);
        motorLeft.set(rightPower);
    }

    @Override
    public void stopMotion() {

    }

    @Override
    public void resetEncoders() {

    }

    @Override
    public void zeroYaw() {

    }

    @Override
    public double[] getEncoderPositions() {
        return new double[0];
    }

    @Override
    public void followPathSimple(Waypoint[] controlPoints) {

    }

    @Override
    public void followPathCSV(String leftTrajectoryFilepath, String rightTrajectoryFilepath) {

    }

}
