package frc.team1285.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team1285.auton.routines.*;
import frc.team1285.loops.*;
import frc.team1285.loops.ClimberControl.ClimberControlState;
import frc.team1285.loops.HopperControl.HopperControlState;
import frc.team1285.loops.IntakeControl.IntakeControlState;
import frc.team1285.loops.ShooterControl.ShooterControlState;
import frc.team1285.loops.SwerveControl.SwerveControlState;
import frc.team1285.util.OI;
import frc.team1285.vision.Limelight;
import frc.team254.lib.trajectory.TrajectoryGenerator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // OI Class
  public static OI oi;

  // Subsystem Control
  public static SwerveControl swerve;
  public static HopperControl hopper;
  public static IntakeControl intake;
  public static ShooterControl shooter;
  public static ClimberControl climber;
  public static RobotState robotState;
  public static RobotControl robotControl;

  // Trajectory Control
  public static TrajectoryGenerator trajectoryGenerator;
  public static QuinticPathTransmitter qTransmitter;

  public static Limelight limelight;

  // Preferences
  Preferences prefs;
  public static double kP_SHOOTER, kI_SHOOTER, kD_SHOOTER, kF_SHOOTER;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Control Instances
    oi = OI.getInstance();
    swerve = SwerveControl.getInstance();
    hopper = HopperControl.getInstance();
    intake = IntakeControl.getInstance();
    shooter = ShooterControl.getInstance();
    climber = ClimberControl.getInstance();
    robotState = RobotState.getInstance();
    robotControl = new RobotControl();

    limelight = Limelight.getInstance();
    limelight.setPipeline(1);
    limelight.setLEDMode(Limelight.OFF);

    trajectoryGenerator = TrajectoryGenerator.getInstance();
    trajectoryGenerator.generateTrajectories();

    qTransmitter = QuinticPathTransmitter.getInstance();

    // Preferences instance
    prefs = Preferences.getInstance();

    // Add Auto Modes
    m_chooser.addOption("Defualt", new WaitCommand(5));

    SmartDashboard.putData("Auto Modes", m_chooser);

    // Reset Sensors
    swerve.zeroSensors();
    swerve.stop();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    updateSmartDashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    updateSmartDashboard();
    // drive.setCoastMode();
    // drive.setBrakeMode();
    limelight.setLEDMode(Limelight.OFF);
  }

  @Override
  public void autonomousInit() {
    swerve.zeroSensors();
    swerve.setNominalDriveOutput(0.0);
    swerve.requireModuleConfiguration();

    shooter.setControlState(ShooterControlState.AUTO_LOOP);
    intake.setControlState(IntakeControlState.AUTO_LOOP);
    hopper.setControlState(HopperControlState.AUTO_LOOP);
    climber.setControlState(ClimberControlState.AUTO_LOOP);
    intake.extendIntake();

    m_autonomousCommand = m_chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    updateSmartDashboard();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    swerve.setNominalDriveOutput(0.0);
    swerve.set10VoltRotationMode(false);

    swerve.setControlState(SwerveControlState.NEUTRAL);
    shooter.setControlState(ShooterControlState.OPEN_LOOP);
    intake.setControlState(IntakeControlState.OPEN_LOOP);
    hopper.setControlState(HopperControlState.OPEN_LOOP);
    climber.setControlState(ClimberControlState.OPEN_LOOP);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    robotControl.update();
    updateSmartDashboard();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void updateSmartDashboard() {

    // Drive
    swerve.outputTelemetry();

    SmartDashboard.putNumber("Right Hopper Encoder", hopper.getRightVelocity());
    SmartDashboard.putNumber("Left Hopper Encoder", hopper.getLeftVelocity());
    SmartDashboard.putNumber("Shooter LEFT RPM", shooter.getRPM());
    SmartDashboard.putNumber("Shooter RIGHT RPM", shooter.getRightRPM());

    SmartDashboard.putNumber("Drive left axis", oi.drivePad.getLeftY());
    SmartDashboard.putNumber("Drive right axis", oi.drivePad.getRightY());

    // kP_SHOOTER = prefs.getDouble("kP_SHOOTER", NumberConstants.pTalonShooter);
    // kI_SHOOTER = prefs.getDouble("kI_SHOOTER", NumberConstants.iTalonShooter);
    // kD_SHOOTER = prefs.getDouble("kD_SHOOTER", NumberConstants.dTalonShooter);
    // if(oi.toolPad.getRightX() > 0.5){
    // shooter.setPIDS(kP_SHOOTER, kI_SHOOTER, kD_SHOOTER);
    // }

    SmartDashboard.putNumber("Limelight X", limelight.getTargetX());
    SmartDashboard.putNumber("Limelight Angle", limelight.pixelToDegree());

    SmartDashboard.putBoolean("Left Climber Sensor", climber.getLeftClimberSwitch());
    SmartDashboard.putBoolean("Right Climber Sensor", climber.getRightClimberSwitch());
  }

}
