// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  /* Initialize the Intake Motor */
  private TalonFX intakeWheelMotor = new TalonFX(IntakeConstants.intakeWheelMotorID);

  private TalonFX intakeArmMotor = new TalonFX(IntakeConstants.intakeArmMotorID);

  private MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  public IntakeSubsystem() {
    /* Configure the motor's factor default settings. */
    intakeWheelMotor.getConfigurator().apply(IntakeConstants.wheelConfigs);
    
    /* Apply a current configuration to the motor. */
    intakeWheelMotor.getConfigurator().refresh(IntakeConstants.currentLimits);
    intakeWheelMotor.getConfigurator().apply(IntakeConstants.currentLimits);
  }

  /**
   * Method to run the intake at a particular setpoint.
   * @param setpoint the percent output at which the intake should run.
   */
  public void runIntake(double setpoint) {
    intakeWheelMotor.set(setpoint);
  }

  public void setIntakePosition(double degrees){
    intakeArmMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  public double getIntakePosition(){
    return Units.rotationsToDegrees(intakeArmMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Percent Output", intakeWheelMotor.get());
  }
}
