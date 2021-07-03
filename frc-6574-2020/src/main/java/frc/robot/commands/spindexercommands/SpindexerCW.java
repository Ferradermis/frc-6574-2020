// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.spindexercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SpindexerCW extends CommandBase {
  /** Creates a new SpindexerCW. */
  double revs;
  double startPosition;
  public SpindexerCW(double revolutions) {
    // Use addRequirements() here to declare subsystem dependencies.
    revs = revolutions;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPosition = RobotContainer.hopper.hopperEncoder.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.hopper.reverseForIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.hopper.hopperEncoder.getPosition() < startPosition - (revs * 100)){
      return true;
    }
    return false;
  }
}
