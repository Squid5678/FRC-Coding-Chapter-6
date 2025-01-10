// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAtRPMCommand extends Command {

  private ShooterSubsystem shooter;
  private double RPM;

  /** Creates a new ShootAtRPMCommand. */
  public ShootAtRPMCommand(ShooterSubsystem shooter, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);

    this.RPM = RPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("DID SHOOT?", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runShooterVelocity(RPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (shooter.getRPM() >= RPM){
          SmartDashboard.putBoolean("DID SHOOT?", true);
    }
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getRPM() >= RPM;
  }
}
