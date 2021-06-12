// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveDistance extends CommandBase {
  /** Creates a new DriveDistance. */
  private int distance;
  private int threshold = 100;

  public DriveDistance(int distance) {
    this.distance = distance;
    addRequirements(RobotContainer.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveTrain.resetPosition();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveTrain.setPosition(distance * RobotContainer.driveTrain.EncoderUnitsPerFeet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int targetUnits = distance * RobotContainer.driveTrain.EncoderUnitsPerFeet;

    if (Math.abs(targetUnits - RobotContainer.driveTrain.getPosition()) < threshold){
      return true;
    }
    if(targetUnits < 0 && RobotContainer.driveTrain.getPosition() < targetUnits){
      return true;
    }
    if(targetUnits > 0 && RobotContainer.driveTrain.getPosition() > targetUnits){
      return true;
    }
    return false;
  }
}
