package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;

public class SetTargetHeadingAction extends CommandBase {
	double targetHeading;
	SwerveControl swerve;

	public SetTargetHeadingAction(double targetHeading) {
		this.targetHeading = targetHeading;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerve = SwerveControl.getInstance();
		swerve.setAbsolutePathHeading(targetHeading);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}

}
