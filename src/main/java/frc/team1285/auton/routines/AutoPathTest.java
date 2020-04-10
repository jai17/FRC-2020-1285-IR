/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1285.auton.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team1285.auton.drive.actions.ResetPoseAction;
import frc.team1285.auton.drive.actions.SetTrajectoryAction;
import frc.team1285.auton.drive.actions.WaitForHeadingAction;
import frc.team1285.auton.drive.actions.WaitToPassXCoordinateAction;
import frc.team1285.loops.SwerveControl;
import frc.team1285.robot.RobotState;
import frc.team1285.util.NumberConstants;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.trajectory.TrajectoryGenerator;

public class AutoPathTest {

    SwerveControl swerve;
    RobotState robotState;

    private double directionFactor;

    private static TrajectoryGenerator.TrajectorySet trajectories;

    public Command get(boolean left) {

        swerve = SwerveControl.getInstance();
        robotState = RobotState.getInstance();

        trajectories = TrajectoryGenerator.getInstance().getTrajectorySet();

        directionFactor = left ? -1.0 : 1.0;

        double startTime = Timer.getFPGATimestamp();

        return new SequentialCommandGroup(new ResetPoseAction(
                left ? new Pose2d(NumberConstants.kRobotLeftStartingPose.getTranslation(), Rotation2d.fromDegrees(90.0))
                        : new Pose2d(NumberConstants.kRobotRightStartingPose.getTranslation(),
                                Rotation2d.fromDegrees(-90.0))),
                new SetTrajectoryAction(trajectories.startToCloseShip.get(left), -90.0 * directionFactor, 1.0),
                new WaitToPassXCoordinateAction((96.0 + NumberConstants.kRobotWidth)),
                new InstantCommand(() -> swerve.setXCoordinate(NumberConstants.closeShipPosition.getTranslation().x())),
                new WaitCommand(0.25),
                new SetTrajectoryAction(trajectories.closeShipToHumanLoader.get(left), -180.0 * directionFactor, 0.75),
                new WaitToPassXCoordinateAction(96.0), new WaitForHeadingAction(160.0, 190.0),
                new InstantCommand(() -> swerve.setYCoordinate(
                        directionFactor * -1.0 * NumberConstants.humanLoaderPosition.getTranslation().y())),
                new InstantCommand(() -> swerve.setXCoordinate(NumberConstants.kRobotHalfLength)),
                new WaitCommand(0.25),
                new SetTrajectoryAction(trajectories.closeShipToHumanLoader.get(left), -180.0 * directionFactor, 1.0),
                new InstantCommand(
                        () -> swerve.setRobotCentricTrajectory(new Translation2d(-36.0, 0.0), -90.0 * directionFactor)),
                new InstantCommand(() -> System.out.println("Auto mode finished in " + startTime + " seconds"))

        );
    }
}