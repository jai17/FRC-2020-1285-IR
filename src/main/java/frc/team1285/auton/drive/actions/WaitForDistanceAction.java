/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;
import frc.team254.lib.geometry.Translation2d;

/**
 * 
 */
public class WaitForDistanceAction extends CommandBase {

    SwerveControl swerve;
    Translation2d pointOfInterest;
    double distance;

    public WaitForDistanceAction(Translation2d target, double distance) {
        pointOfInterest = target;
        this.distance = distance;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve = SwerveControl.getInstance();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return swerve.getPose().getTranslation().distance(pointOfInterest) <= distance;
    }

}
