package frc.team2974.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team2974.robot.command.teleop.Drive;

import static frc.team2974.robot.Config.RobotConstants.DISTANCE_PER_PULSE;
import static frc.team2974.robot.RobotMap.*;

public class Drivetrain extends Subsystem {

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

    public void ZeroYaw() {
        ahrs.zeroYaw();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive());
    }

    public void reset() {
        encoderLeft.reset();
        encoderRight.reset();
    }

    public void setSpeeds(double leftPower, double rightPower) {
        motorRight.set(leftPower);
        motorLeft.set(rightPower);
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

}
