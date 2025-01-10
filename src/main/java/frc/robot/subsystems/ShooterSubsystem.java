// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final FlywheelSim shooterSimModel = new FlywheelSim(DCMotor.getKrakenX60(2), 1, 0.006);

  private TalonFX shooterPrimary = new TalonFX(ShooterConstants.shooterPrimaryID);
  private TalonFX shooterFollower = new TalonFX(ShooterConstants.shooterFollowerID);

  private VelocityVoltage velocityRequest;

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

    velocityRequest  = new VelocityVoltage(0);

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

  public void runShooterVelocity(double RPM){
      shooterPrimary.setControl(velocityRequest.withVelocity(RPM * ShooterConstants.RPMtoRPS));
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

  public double getRPM(){
    return shooterPrimary.getVelocity().getValueAsDouble() * ShooterConstants.RPStoRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("SHOOTER PERCENT OUT", getPercentOut());

    SmartDashboard.putNumber("SHOOTER RPM", getRPM());
  }

  @Override
  public void simulationPeriodic() {
   var talonFXSim = shooterPrimary.getSimState();

   // set the supply voltage of the TalonFX
   talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
   var motorVoltage = talonFXSim.getMotorVoltage();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   shooterSimModel.setInputVoltage(motorVoltage);
   shooterSimModel.update(0.020); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   talonFXSim.setRotorVelocity(Units.radiansToRotations(shooterSimModel.getAngularVelocityRadPerSec())
   );
}
}
