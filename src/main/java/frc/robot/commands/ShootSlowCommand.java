// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSlowCommand extends Command {

  private final double slowPercent = 0.1;

  private final double timeout = 5; //Challenge

  private ShooterSubsystem shooter;

  private Timer timer = new Timer(); //Challenge
  
  /** Creates a new ShootCommand. */
  public ShootSlowCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.restart(); //Challenge
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runShooter(slowPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5); //Challenge
    //return false; //Normal
  }
}
