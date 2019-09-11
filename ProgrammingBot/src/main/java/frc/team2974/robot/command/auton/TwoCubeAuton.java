package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TwoCubeAuton extends CommandGroup {

    public TwoCubeAuton() {
        addSequential(new GeneralCurve());
        addSequential(new WaitForTx());
        addSequential(new VisionLine());
    }

    @Override
    protected void interrupted() {
        System.out.println("Two cube auton interrupted.");
        end();
    }

}
