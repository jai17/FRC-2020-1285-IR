package frc.team1285.auton.drive.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1285.loops.SwerveControl;
import frc.team254.lib.geometry.Pose2dWithCurvature;
import frc.team254.lib.trajectory.Trajectory;
import frc.team254.lib.trajectory.timing.TimedState;

public class SetTrajectoryAction extends CommandBase {

	Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
	double goalHeading;
	double rotationScalar;
	SwerveControl swerve;

	public SetTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading,
			double rotationScalar) {
		this.trajectory = trajectory;
		this.goalHeading = goalHeading;
		this.rotationScalar = rotationScalar;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerve = SwerveControl.getInstance();
		swerve.setTrajectory(trajectory, goalHeading, rotationScalar);
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
