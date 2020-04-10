package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;

public class WaitToFinishPathAction extends CommandBase {

	SwerveControl swerve;
	double timeout;
	double startTime;

	public WaitToFinishPathAction() {
		timeout = 15.0;
	}

	public WaitToFinishPathAction(double timeout) {
		this.timeout = timeout;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerve = SwerveControl.getInstance();
		startTime = Timer.getFPGATimestamp();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return swerve.hasFinishedPath() || ((Timer.getFPGATimestamp() - startTime) > timeout);
	}
}
