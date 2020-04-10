/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1285.auton.sequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team1285.loops.HopperControl;
import frc.team1285.loops.IntakeControl;
import frc.team1285.loops.ShooterControl;
import frc.team1285.robot.Robot;

/**
 * Add your docs here.
 */
public class Sequences {

    private ShooterControl shooter;
    private IntakeControl intake;
    private HopperControl hopper;

    private Command currentCommand;

    public Sequences() {
        shooter = ShooterControl.getInstance();
        intake = IntakeControl.getInstance();
        hopper = HopperControl.getInstance();
    }

    public SequentialCommandGroup IntakeSequence() {
        return new SequentialCommandGroup(new InstantCommand(() -> intake.setIntakeSpeed(0.8)),
                new InstantCommand(() -> hopper.setLeftHopperSpeed(-1.0)),
                new InstantCommand(() -> hopper.setRightHopperSpeed(0.70)),
                new InstantCommand(() -> shooter.setConveyorSpeed(-0.25)));
    }

    public SequentialCommandGroup ShortShootSequence() {
        return new SequentialCommandGroup(new InstantCommand(() -> shooter.setConveyorSpeed(0.8)));
    }

    public SequentialCommandGroup ShootSequence() {
        return new SequentialCommandGroup(new InstantCommand(() -> hopper.setLeftHopperSpeed(-1.0)),
                new InstantCommand(() -> hopper.setRightHopperSpeed(0.70)),
                new InstantCommand(() -> shooter.setConveyorSpeed(0.8)),
                new InstantCommand(() -> intake.setIntakeSpeed(0.5)));
    }

    public SequentialCommandGroup Stop() {
        return new SequentialCommandGroup(new InstantCommand(() -> shooter.setRPM(0)),
                new InstantCommand(() -> hopper.setLeftHopperSpeed(0)),
                new InstantCommand(() -> hopper.setRightHopperSpeed(0)),
                new InstantCommand(() -> shooter.setConveyorSpeed(0)));
    }

    public SequentialCommandGroup StopDrive() {
        return new SequentialCommandGroup(new InstantCommand(() -> Robot.swerve.stop()));
    }

    public boolean isFinished() {
        if (currentCommand == null) {
            return true;
        }
        return currentCommand.isFinished();
    }

    public void startCommand(Command newCommand) {
        if (currentCommand != null) {
            currentCommand.cancel();
        }

        currentCommand = newCommand;
        currentCommand.schedule();
    }
}
