/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2974.robot.command;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import static frc.team2974.robot.Robot.drivetrain;

public class AutoCommandGroup extends CommandGroup {

    public AutoCommandGroup(int choice) {
        requires(drivetrain);

        System.out.println("AutoCommandGroup choice:  " + choice);
        switch (choice) {
            case 1:
                addSequential(new AutoSpin(90));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoSpin(180));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoSpin(-90));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoRotate(0));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoSpin(90));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoSpin(180));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoSpin(-90));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoRotate(0));
                break;
            case 2:
                addSequential(new AutoDriveToTarget(this));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoDriveDistance(-0.5));
                addSequential(new AutoSpin(180));
                addSequential(new AutoDriveDistance(1.5));
                addSequential(new AutoSpin(180));
                addSequential(new AutoDriveDistance(0.75));
                addSequential(new AutoDriveToTarget(this));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoDriveDistance(-0.5));
                addSequential(new AutoSpin(180));
                addSequential(new AutoDriveDistance(1.5));
                addSequential(new AutoSpin(180));
                addSequential(new AutoDriveDistance(0.75));
                addSequential(new AutoDriveToTarget(this));
                break;
            case 3:
                addSequential(new AutoDriveToTarget(this));
                addSequential(new WaitCommand(0.25));
                addSequential(new AutoDriveDistance(-0.5));
                addSequential(new AutoRotate(170));
                addSequential(new AutoDriveDistance(0.5));
                addSequential(new AutoRotate(200));
                addSequential(new AutoDriveDistance(0.5));
                addSequential(new AutoRotate(0));
                addSequential(new AutoDriveDistance(0.3));
                addSequential(new AutoDriveToTarget(this));
                break;
            case 4:
                addSequential(new AutoDriveToTarget(this));
                break;
        }

        // these will run in order.
        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        // addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }

    // Exit if one of the commands cancels
    @Override
    protected void interrupted() {
        System.out.println("AutoCommandGroup cancelled (see previous error)");
        end();
    }
}
