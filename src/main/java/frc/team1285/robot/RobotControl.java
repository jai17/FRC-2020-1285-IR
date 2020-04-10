package frc.team1285.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team1285.loops.ShooterControl;
import frc.team1285.loops.SwerveControl;
import frc.team1285.loops.ShooterControl.ShooterControlState;
import frc.team1285.util.NumberConstants;
import frc.team1285.util.OI;

public class RobotControl {

    private OI oi;

    private SwerveControl swerve;
    private ShooterControl shooter;

    private Trigger shootTrigger;

    private boolean flickRotation = false;
    private boolean robotCentric = false;

    public RobotControl() {
        oi = OI.getInstance();

        swerve = SwerveControl.getInstance();
        shooter = ShooterControl.getInstance();

        shootTrigger = new Trigger(oi.toolPad::getRightTrigger);

        // forwardTurretTrigger = new Trigger(() -> oi.getToolRightY() > 0.9);

        shootTrigger.whenActive(new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setControlState(ShooterControlState.OPEN_LOOP)),
                new InstantCommand(() -> shooter.setRPM(3800))));
        shootTrigger.whenInactive(new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setControlState(ShooterControlState.OPEN_LOOP)),
                new InstantCommand(() -> shooter.setRPM(0))));

    }

    public void update() {

        double swerveYInput = oi.drivePad.getLeftX();
        double swerveXInput = -oi.drivePad.getLeftY();
        double swerveRotationInput = (flickRotation ? 0.0 : oi.drivePad.getRightX());

        if (oi.drivePad.getBackButton()) {
            swerve.temporarilyDisableHeadingController();
            swerve.zeroSensors(NumberConstants.kRobotLeftStartingPose);
        }

        if (oi.drivePad.getXButton())
            swerve.rotate(0);
        else if (oi.drivePad.getAButton())
            swerve.rotate(90);
        else if (oi.drivePad.getBButton())
            swerve.rotate(180);
        else if (oi.drivePad.getYButton())
            swerve.rotate(270);

        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric,
                oi.drivePad.getLeftTriggerPressed());

    }
}