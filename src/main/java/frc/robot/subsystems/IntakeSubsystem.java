// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  /* Initialize the Intake Motor */
  private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);

  public IntakeSubsystem() {
    /* Configure the motor's factor default settings. */
    intakeMotor.getConfigurator().apply(IntakeConstants.configs);
    
    /* Apply a current configuration to the motor. */

    System.out.println(IntakeConstants.currentLimits.StatorCurrentLimit);

    intakeMotor.getConfigurator().refresh(IntakeConstants.currentLimits);

    System.out.println(IntakeConstants.currentLimits.StatorCurrentLimit);

    intakeMotor.getConfigurator().apply(IntakeConstants.currentLimits);
  }

  /**
   * Method to run the intake at a particular setpoint.
   * @param setpoint the percent output at which the intake should run.
   */
  public void runIntake(double setpoint) {
    intakeMotor.set(setpoint);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Percent Output", intakeMotor.get());
  }
}
