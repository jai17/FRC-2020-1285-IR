package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;

public class WaitToPassYCoordinateAction extends CommandBase {

	SwerveControl swerve;
	double startingYCoordinate;
	double targetYCoordinate;

	public WaitToPassYCoordinateAction(double y) {
		targetYCoordinate = y;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerve = SwerveControl.getInstance();
		startingYCoordinate = swerve.getPose().getTranslation().y();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.signum(startingYCoordinate - targetYCoordinate) != Math
				.signum(swerve.getPose().getTranslation().y() - targetYCoordinate);
	}
}
