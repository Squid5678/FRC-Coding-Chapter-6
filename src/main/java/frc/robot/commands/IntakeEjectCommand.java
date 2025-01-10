// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeEjectCommand extends Command {
  /** Creates a new IntakeEjectCommand. */

  /* Declare the intake subsytem */
  private IntakeSubsystem intakeSubsystem;

  public IntakeEjectCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.runIntake(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
