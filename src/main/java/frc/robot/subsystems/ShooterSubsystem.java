// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX shooterPrimary = new TalonFX(ShooterConstants.shooterPrimaryID);
  private TalonFX shooterFollower = new TalonFX(ShooterConstants.shooterFollowerID);

  /*
   * CONFIG SECTION
   */
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    
    /* Config to our default */
    shooterPrimary.getConfigurator().apply(ShooterConstants.configs);
    shooterFollower.getConfigurator().apply(ShooterConstants.configs);

    /*Apply current limits */
    applyCurrentLimits();

    /* Follower setup */
    shooterFollower.setControl(new Follower(ShooterConstants.shooterPrimaryID, false));

  }

  /* Check Tank Drive Subsystem for more detailed comments */
  public void applyCurrentLimits(){

    CurrentLimitsConfigs currentConfigs = ShooterConstants.currentLimits;

    shooterPrimary.getConfigurator().refresh(currentConfigs);
    shooterFollower.getConfigurator().refresh(currentConfigs);

    shooterPrimary.getConfigurator().apply(currentConfigs);
    shooterFollower.getConfigurator().apply(currentConfigs);
  }


  /*
   * CONTROLLING SECTION
   */

  public void runShooter(double percent){
    shooterPrimary.set(percent);
  }

  public void stop(){
    shooterPrimary.stopMotor();
  }


  /*
   * PRINTING SECTION
   */

  public double getPercentOut(){
    return shooterPrimary.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("SHOOTER PERCENT OUT", getPercentOut());
  }
}
