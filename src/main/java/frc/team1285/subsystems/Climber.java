/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1285.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1285.util.ElectricalConstants;

public class Climber extends SubsystemBase {


  private DoubleSolenoid climberSolenoid;

  DigitalInput leftClimberInput;
  DigitalInput rightClimberInput;

  public Climber() {

    climberSolenoid = new DoubleSolenoid(ElectricalConstants.CLIMBER_A, ElectricalConstants.CLIMBER_B);

    leftClimberInput = new DigitalInput(ElectricalConstants.LEFT_CLIMBER_SENSOR);
    rightClimberInput = new DigitalInput(ElectricalConstants.RIGHT_CLIMBER_SENSOR);
  }

  public void unlatchClimber() {
    this.climberSolenoid.set(Value.kForward);
  }

  public void latchClimber() {
    this.climberSolenoid.set(Value.kReverse);
  }

  public boolean getClimberLatched() {
    return this.climberSolenoid.get() == Value.kReverse ? true : false;
  }

  /**
   * Checks if left climber detects magnet
   * 
   * @return true if sensor detects
   */
  public boolean getLeftClimberSwitch() {
    // Digital Sensors are reversed
    return !leftClimberInput.get();
  }

  /**
   * Checks if right climber detect magnet
   * 
   * @return true if sensor detects
   */
  public boolean getRightClimberSwitch() {
    // Digital Sensors are reversed
    return !rightClimberInput.get();
  }

}
