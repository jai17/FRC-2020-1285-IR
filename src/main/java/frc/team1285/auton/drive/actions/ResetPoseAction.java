package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;
import frc.team1285.util.NumberConstants;
import frc.team254.lib.geometry.Pose2d;

public class ResetPoseAction extends CommandBase {

	private Pose2d newPose;
	boolean leftStartingSide = true;

	SwerveControl swerve;

	public ResetPoseAction(Pose2d newPose) {
		this.newPose = newPose;
	}

	public ResetPoseAction(boolean left) {
		newPose = left ? NumberConstants.kRobotLeftStartingPose : NumberConstants.kRobotRightStartingPose;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerve = SwerveControl.getInstance();
		swerve.setStartingPose(newPose);
		swerve.zeroSensors(newPose);
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
