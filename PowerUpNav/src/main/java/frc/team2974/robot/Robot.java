package frc.team2974.robot;

//import static frc.team2974.robot.RobotMap.pneumaticsShifter;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.command.AutoCommandGroup;
import frc.team2974.robot.subsystems.Drivetrain;
import org.waltonrobotics.MotionLogger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static MotionLogger motionLogger;
    public static OI oi;
    private static Config.Robot currentRobot;
    private int dashCounter = 0;
    private Command autoCommand;
    private SendableChooser<AutoCommandGroup> autoMode;

    public static Config.Robot getChoosenRobot() {
        return currentRobot;
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        //currentRobot = RobotMap.robotIdentifier.get() ? Config.Robot.COMPETITION : Config.Robot.PRACTICE;
        currentRobot = Config.Robot.PRACTICE;

        motionLogger = new MotionLogger("/home/lvuser/");
        drivetrain = new Drivetrain(motionLogger);
        oi = new OI();
        autoMode = new SendableChooser<>();
        autoMode.setDefaultOption("Automode 1", new AutoCommandGroup(1));
        autoMode.addOption("Automode 2", new AutoCommandGroup(2));
        autoMode.addOption("Spline 1", new AutoCommandGroup(3));
        autoMode.addOption("Spline 2", new AutoCommandGroup(4));
        SmartDashboard.putData("AutoCommandGroup", autoMode);
        drivetrain.shiftDown();
        System.out.println("Robot initializing...");
    }

    @Override
    public void disabledInit() {
        drivetrain.cancelControllerMotion();
        drivetrain.reset();
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autoCommand = autoMode.getSelected();
        autoCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        drivetrain.updateLimelightTracking();
        Scheduler.getInstance().run();
        updateSmartDashboard("Auto");
    }

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
        drivetrain.cancelControllerMotion();
        drivetrain.shiftUp(); // start in high gear
        drivetrain.reset();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        drivetrain.updateLimelightTracking();
        Scheduler.getInstance().run();
        updateSmartDashboard("TeleOp");
    }

    @Override
    public void testInit() {
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * Put things in here you want to update for SmartDashboard.
     */
    private void updateSmartDashboard(String caller) {
        // only update every 50th call to save CPU and bandwidth
        if (dashCounter == 50) {
            dashCounter = 0;
        } else {
            dashCounter++;
            return;
        }

        //System.out.print(caller + ": " + java.time.LocalTime.now() + " R dist: " + -RobotMap.encoderLeft.getDistance() + " L dist: " + -RobotMap.encoderRight.getDistance());
        //System.out.print(" Heading: " +  drivetrain.ahrs.getYaw() + " Rotate rate: " + drivetrain.rotateToAngleRate);
        //System.out.println(" 3D LR offset: " + drivetrain.camtran[0] + " 3D dist offset: " + drivetrain.camtran[2]);
        SmartDashboard.putNumber("R_Enc_Dist", -RobotMap.encoderLeft.getDistance());
        SmartDashboard.putNumber("L_Enc_Dist", -RobotMap.encoderRight.getDistance());

        // Drivetrain
        //SmartDashboard.putString("Gear", pneumaticsShifter.get() ? "Low" : "High");

        /* navX */
        // Angle is total degrees including rotations (useful for a turret)
        //SmartDashboard.putNumber("IMU_Angle", ahrs.getAngle());
        SmartDashboard.putNumber("IMU_Yaw", drivetrain.ahrs.getYaw());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        //SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
        //SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

        /* Limelight */
        //SmartDashboard.putNumber(   "Limelight tv",          tv);
        SmartDashboard.putNumber("Limelight tx", drivetrain.tx);
        SmartDashboard.putNumber("Limelight ty", drivetrain.ty);
        SmartDashboard.putNumber("Limelight ta", drivetrain.ta);
        SmartDashboard.putBoolean("Limelight target?", drivetrain.limelightHasValidTarget);
        SmartDashboard.putNumber("Limelight 3D x", drivetrain.camtran[0]);
        //SmartDashboard.putNumber("Limelight 3D y", drivetrain.camtran[1]);
        SmartDashboard.putNumber("Limelight 3D z", drivetrain.camtran[2]);
        SmartDashboard.putNumber("Limelight 3D pitch", drivetrain.camtran[3]);
        SmartDashboard.putNumber("Limelight 3D yaw", drivetrain.camtran[4]);
        SmartDashboard.putNumber("Limelight 3D roll", drivetrain.camtran[5]);
    }
}
  