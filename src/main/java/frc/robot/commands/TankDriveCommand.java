// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDriveSubsystem;

public class TankDriveCommand extends Command {

  /* Our one subsystem object*/
  private TankDriveSubsystem driveSubsystem;

  /* Suppliers to get the most up-to-date controlling values  */
  private DoubleSupplier leftSupplier;
  private DoubleSupplier rightSupplier;

  /** Creates a new TankDriveCommand. */
  public TankDriveCommand(TankDriveSubsystem driveSubsystem, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveSubsystem = driveSubsystem;
    addRequirements(this.driveSubsystem);


    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // driveSubsystem.drive(leftSupplier.getAsDouble(), rightSupplier.getAsDouble()); //No deadband
    double leftPercentDeadbanded = MathUtil.applyDeadband(leftSupplier.getAsDouble(), 0.07);
    double rightPercentDeadbanded = MathUtil.applyDeadband(rightSupplier.getAsDouble(), 0.07);


    driveSubsystem.drive(leftPercentDeadbanded, rightPercentDeadbanded);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
