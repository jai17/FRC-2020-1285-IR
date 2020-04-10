package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;

/**
 * An action designed to wait until there is only a certain (specified) amount
 * of time left before the completion of a trajectory.
 */
public class RemainingProgressAction extends CommandBase {

    SwerveControl swerve;
    double targetProgress = 0.0;

    public RemainingProgressAction(double targetProgress) {
        this.targetProgress = targetProgress;
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
        return swerve.getRemainingProgress() <= targetProgress;
    }
}
