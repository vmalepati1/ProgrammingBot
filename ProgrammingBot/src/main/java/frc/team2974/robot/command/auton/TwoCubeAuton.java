package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TwoCubeAuton extends CommandGroup {

    public TwoCubeAuton() {
        addSequential(new DriveToRocket());
        addSequential(new VisionCorrection());
    }

    @Override
    protected void interrupted() {
        System.out.println("Two cube auton interrupted.");
        end();
    }

}
