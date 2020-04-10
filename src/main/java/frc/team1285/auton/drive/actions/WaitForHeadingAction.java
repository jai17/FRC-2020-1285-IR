package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;

public class WaitForHeadingAction extends CommandBase {

	SwerveControl swerve;
	double lowThreshold;
	double highThreshold;

	public WaitForHeadingAction(double lowThreshold, double highThreshold) {
		this.lowThreshold = lowThreshold;
		this.highThreshold = highThreshold;
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
		double heading = swerve.getPose().getRotation().getUnboundedDegrees();
		return heading >= lowThreshold && heading <= highThreshold;
	}

}
