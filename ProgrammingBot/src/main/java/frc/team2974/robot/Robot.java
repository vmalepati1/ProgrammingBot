package frc.team2974.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.command.AbsolutePoseTracker;
import frc.team2974.robot.command.auton.PathfinderSplineTest;
import frc.team2974.robot.subsystems.Drivetrain;

import javax.management.*;
import java.lang.management.ManagementFactory;

import static frc.team2974.robot.Config.SmartDashboardKeys.TESTING_AUTON_SELECT;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static OI oi;
    public static NetworkTable waltonDashboard;

    private SendableChooser<Command> autonSendableChooser;

    @Override
    public void robotInit() {
        System.out.println("Initializing robot.");

        drivetrain = new Drivetrain();
        oi = new OI();
        waltonDashboard = NetworkTableInstance.getDefault().getTable("WaltonDashboard");

        autonSendableChooser = new SendableChooser<>();
        autonSendableChooser.setDefaultOption("Pathfinder Spline Test", new PathfinderSplineTest());
        SmartDashboard.putData(TESTING_AUTON_SELECT, autonSendableChooser);

        drivetrain.reset();
        drivetrain.shiftDown();

        AbsolutePoseTracker.setStartingPose(new double[]{0, 0, Math.PI / 2});
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
        waltonDashboard.getEntry("Diagnostics/Battery Voltage").setNumber(DriverStation.getInstance().getBatteryVoltage());

        try {
            waltonDashboard.getEntry("Diagnostics/RIO CPU Use").setNumber(getRIOCPUUse());
        } catch (MalformedObjectNameException | ReflectionException | InstanceNotFoundException e) {
            e.printStackTrace();
        }
        waltonDashboard.getEntry("Diagnostics/RIO RAM Use").setNumber(getRIORamUse());

    }

    @Override
    public void disabledInit() {
        new AbsolutePoseTracker().start();
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        new AbsolutePoseTracker().start();

        drivetrain.shiftDown();

        autonSendableChooser.getSelected().start();
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        new AbsolutePoseTracker().start();

        autonSendableChooser.getSelected().cancel();

        drivetrain.shiftUp();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }

}
  