/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;

public class WaitToLeaveRampAction extends CommandBase {

    SwerveControl swerve;
    boolean startedDescent = false;
    double startingRoll = 0.0;
    double timeout = 1.0;
    boolean timedOut = false;

    public boolean timedOut() {
        return timedOut;
    }

    double startTime = 0.0;

    final double kAngleTolerance = 1.5;
    final double kMinExitAngle = 5.0;

    public WaitToLeaveRampAction(double timeout) {
        this.timeout = timeout;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve = SwerveControl.getInstance();
        startTime = Timer.getFPGATimestamp();
        startingRoll = swerve.getRoll();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double rollAngle = swerve.getRoll();
        // System.out.println("Pigeon roll: " + rollAngle);
        if (Math.abs(rollAngle) >= (startingRoll + kMinExitAngle) && !startedDescent)
            startedDescent = true;
        if (startedDescent && Math.abs(rollAngle - startingRoll) <= kAngleTolerance) {
            timedOut = false;
            return true;
        }

        if ((Timer.getFPGATimestamp() - startTime) > timeout) {
            timedOut = true;
            System.out.println("WaitToLeaveRampAction timed out");
            return true;
        }

        return false;
    }
}
