package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;

public class WaitToPassXCoordinateAction extends CommandBase {
	double startingXCoordinate;
	double targetXCoordinate;
	SwerveControl swerve;

	public WaitToPassXCoordinateAction(double x) {
		targetXCoordinate = x;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerve = SwerveControl.getInstance();
		startingXCoordinate = swerve.getPose().getTranslation().x();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate) != Math
				.signum(swerve.getPose().getTranslation().x() - targetXCoordinate);
	}

}
