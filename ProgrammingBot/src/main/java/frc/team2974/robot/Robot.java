package frc.team2974.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.command.auton.StraightTuning;
import frc.team2974.robot.command.auton.TwoCubeAuton;
import frc.team2974.robot.command.teleop.Drive;
import frc.team2974.robot.subsystems.Drivetrain;

import javax.management.*;
import java.lang.management.ManagementFactory;

import static frc.team2974.robot.Config.PathfinderConstantsDefaults.*;
import static frc.team2974.robot.Config.SmartDashboardKeys.*;
import static frc.team2974.robot.Config.VisionConstantsDefaults.*;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static OI oi;
    // public static NetworkTable waltonDashboard;
    private SendableChooser<Command> autonSelect;

    private void initShuffleBoard() {
        SmartDashboard.putNumber(PATHFINDER_HEADING, 0);
        SmartDashboard.putNumber(PATHFINDER_DESIRED_HEADING, 0);
        SmartDashboard.putNumber(PATHFINDER_HEADING_DIFFERENCE, 0);
        SmartDashboard.putNumber(PATHFINDER_TURN, 0);
        SmartDashboard.putNumber(PATHFINDER_LEFT_MOTOR_SPEED, 0);
        SmartDashboard.putNumber(PATHFINDER_RIGHT_MOTOR_SPEED, 0);

        SmartDashboard.putNumber(PATHFINDER_LEFT_FOLLOWER_KP, PATHFINDER_LEFT_FOLLOWER_KP_DEFAULT);
        SmartDashboard.putNumber(PATHFINDER_LEFT_FOLLOWER_KD, PATHFINDER_LEFT_FOLLOWER_KD_DEFAULT);
        SmartDashboard.putNumber(PATHFINDER_RIGHT_FOLLOWER_KP, PATHFINDER_RIGHT_FOLLOWER_KP_DEFAULT);
        SmartDashboard.putNumber(PATHFINDER_RIGHT_FOLLOWER_KD, PATHFINDER_RIGHT_FOLLOWER_KD_DEFAULT);

        SmartDashboard.putNumber(ABSOLUTE_POSE_X, 0);
        SmartDashboard.putNumber(ABSOLUTE_POSE_Y, 0);
        SmartDashboard.putNumber(ABSOLUTE_POSE_ANGLE_DEGREES, 0);

        SmartDashboard.putNumber(VISION_STEER_K, VISION_STEER_K_DEFAULT);
        SmartDashboard.putNumber(VISION_DRIVE_K, VISION_DRIVE_K_DEFAULT);
        SmartDashboard.putNumber(VISION_DESIRED_TARGET_AREA, VISION_DESIRED_TARGET_AREA_DEFAULT);
        SmartDashboard.putNumber(VISION_MAX_DRIVE, VISION_MAX_DRIVE_DEFAULT);
        SmartDashboard.putNumber(VISION_ALIGNED_X_PLUS_MINUS, VISION_ALIGNED_X_PLUS_MINUS_DEFAULT);
        SmartDashboard.putNumber(VISION_ALIGNED_Z_PLUS_MINUS, VISION_ALIGNED_Z_PLUS_MINUS_DEFAULT);

        autonSelect = new SendableChooser<>();
        autonSelect.setDefaultOption("Two Cube Auton", new TwoCubeAuton());
        autonSelect.addOption("Straight Tuning", new StraightTuning());
        SmartDashboard.putData(AUTON_COMMAND_GROUP, autonSelect);
    }

    @Override
    public void robotInit() {
        System.out.println("Initializing robot.");

        drivetrain = new Drivetrain();
        oi = new OI();
        // waltonDashboard = NetworkTableInstance.getDefault().getTable("WaltonDashboard");

        drivetrain.resetEncoders();
        drivetrain.shiftDown();

        // AbsolutePoseTracker.setStartingPose(new double[]{0, 0, Math.PI / 2});
        // AbsolutePoseTracker.getInstance().start();

        initShuffleBoard();
    }

    private double getRIOCPUUse() throws MalformedObjectNameException, ReflectionException, InstanceNotFoundException {
        MBeanServer mbs = ManagementFactory.getPlatformMBeanServer();
        ObjectName name = ObjectName.getInstance("java.lang:type=OperatingSystem");
        AttributeList list = mbs.getAttributes(name, new String[]{"ProcessCpuLoad"});

        if (list.isEmpty()) return Double.NaN;

        Attribute att = (Attribute) list.get(0);
        Double value = (Double) att.getValue();

        // usually takes a couple of seconds before we get real values
        if (value == -1.0) return Double.NaN;
        // returns a percentage value with 1 decimal point precision
        return ((int) (value * 1000) / 10.0);
    }

    private double getRIORamUse() {
        long ramTotal = Runtime.getRuntime().totalMemory();
        long ramFree = Runtime.getRuntime().freeMemory();
        long ramUsed = ramTotal - ramFree;

        return ((ramUsed * 1.0) / ramTotal);
    }

    @Override
    public void robotPeriodic() {
        /*
        waltonDashboard.getEntry("Diagnostics/Battery Voltage").setNumber(DriverStation.getInstance().getBatteryVoltage());

        try {
            waltonDashboard.getEntry("Diagnostics/RIO CPU Use").setNumber(getRIOCPUUse());
        } catch (MalformedObjectNameException | ReflectionException | InstanceNotFoundException e) {
            e.printStackTrace();
        }
        waltonDashboard.getEntry("Diagnostics/RIO RAM Use").setNumber(getRIORamUse());
        */

        //drivetrain.updateLimelightTracking();
        SmartDashboard.putNumber("TX", Math.toDegrees(drivetrain.getTx()));
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        drivetrain.shiftDown();

        autonSelect.getSelected().start();
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        new Drive().start();

        drivetrain.shiftUp();
        drivetrain.resetEncoders();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        SmartDashboard.putNumber("Left enc", encoderLeft.get());
        SmartDashboard.putNumber("Right enc", encoderRight.get());
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }

}
  