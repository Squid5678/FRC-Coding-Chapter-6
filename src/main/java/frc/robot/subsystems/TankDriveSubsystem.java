// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TankDriveConstants;

public class TankDriveSubsystem extends SubsystemBase {

  private TalonFX leftPrimary = new TalonFX(TankDriveConstants.leftPrimaryID);
  private TalonFX leftFollower = new TalonFX(TankDriveConstants.leftFollowerID);

  private TalonFX rightPrimary = new TalonFX(TankDriveConstants.rightPrimaryID);
  private TalonFX rightFollower = new TalonFX(TankDriveConstants.rightFollowerID);

  /* Declare Differential Drive object */
  private DifferentialDrive kDrive;


  /*
   * CONFIG SECTION
   */

  /** Creates a new TankDriveSubsystem. */
  public TankDriveSubsystem() {

    /* Using our default configs */
    leftPrimary.getConfigurator().apply(TankDriveConstants.configs);
    leftFollower.getConfigurator().apply(TankDriveConstants.configs);
    rightPrimary.getConfigurator().apply(TankDriveConstants.configs);
    rightFollower.getConfigurator().apply(TankDriveConstants.configs);

    /* Applying current limits */
    applyCurrentLimits();

    /* Inverting one of the sides (tank drive) */
    rightPrimary.setInverted(true);

    /* Following the primary */
    leftFollower.setControl(new Follower(TankDriveConstants.leftPrimaryID, false));
    rightFollower.setControl(new Follower(TankDriveConstants.rightPrimaryID, false));

    /* Initialize Differential Drive object here -- be sure to do so once motor followers are set!
     * (NOTE: this is usually done because of the way other code libraries work with the DifferentialDrive object.)
     */
    kDrive = new DifferentialDrive(leftPrimary, rightPrimary);

  }

  /* Helper method for appolying current limits */
  public void applyCurrentLimits(){
    
    /* Using our limit configs */
    CurrentLimitsConfigs currentConfigs = TankDriveConstants.currentLimits;

    /* Refreshing the motors' current limits */
    leftPrimary.getConfigurator().refresh(currentConfigs);
    leftFollower.getConfigurator().refresh(currentConfigs);
    rightPrimary.getConfigurator().refresh(currentConfigs);
    rightFollower.getConfigurator().refresh(currentConfigs);

    /* Applying our limit configs */
    leftPrimary.getConfigurator().apply(currentConfigs);
    leftFollower.getConfigurator().apply(currentConfigs);
    rightPrimary.getConfigurator().apply(currentConfigs);
    rightFollower.getConfigurator().apply(currentConfigs);
    
  }

  /*
   * CONTROLLING SECTION
   */

  /* Telling the motors to drive */
  public void drive(double leftVelocity, double rightVelocity){
    leftPrimary.set(leftVelocity);
    rightPrimary.set(rightVelocity);
  }

  /* Telling motors to stop */
  public void stop(){
    leftPrimary.stopMotor();
    rightPrimary.stopMotor();
  }

  /**
   * Method to get the DifferentialDrive object.
   * @return a DifferentialDrive object, to be accessed by other commands. 
   */
  public DifferentialDrive getKDrive() {
    return kDrive;
  }

  /* 
   * GETTERS SECTION 
  */

  /* Getting the left side's % out */
  public double getLeftPercentOut(){
    return leftPrimary.get();
  }

  /* Getting the right side's % out */
  public double getRightPercentOut(){
    return rightPrimary.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("LEFT DRIVE % OUT", getLeftPercentOut());
    SmartDashboard.putNumber("RIGHT DRIVE % OUT", getRightPercentOut());
  }
}
