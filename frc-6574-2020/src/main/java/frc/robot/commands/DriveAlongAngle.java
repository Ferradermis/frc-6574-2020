/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveAlongAngle extends CommandBase {
  /**
   * Creates a new DriveAlongDistance.
   */

  double kF = 0.1;  //kF is essentially minimal amount to drive
  double kP = 0.75;
  double tolerance = 750; // this would be roughly 1 inch

  double angleKP = .005;
  
  final static double DefaultMaxDriveSpeed = 0.45;
  final double MaxTurnSpeed = 0.25;
  final double EncoderUnitsPerFeet = 14500;

  double maxDriveSpeed;
  double driveSpeed;
  double turnSpeed = 0.0;
  double distanceInFeet;
  int direction;
  double alongAngle;
  DriveTrain driveTrain;
  double distanceError;
  double endPosition;
  double angleError;

  public DriveAlongAngle(double distanceInFeet, double alongAngle)
  {
    this(distanceInFeet, alongAngle, DefaultMaxDriveSpeed);
  }

  public DriveAlongAngle(double distanceInFeet, double alongAngle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);
    this.distanceInFeet = distanceInFeet;
    this.direction = (distanceInFeet > 0) ? 1 : -1;;
    this.alongAngle = alongAngle;
    maxDriveSpeed = speed;
    driveTrain = RobotContainer.driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceError = distanceInFeet * EncoderUnitsPerFeet * direction;    
    endPosition = driveTrain.getPosition() + distanceError;

    angleError = alongAngle-driveTrain.getGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSpeed = distanceError / EncoderUnitsPerFeet / 5 * kP + Math.copySign(kF,distanceError);

    // make sure we go no faster than maxDriveSpeed
    driveSpeed = ((Math.abs(driveSpeed) > maxDriveSpeed) ? Math.copySign(maxDriveSpeed, driveSpeed) :  driveSpeed);

    angleError = alongAngle-driveTrain.getGyroAngle();
    turnSpeed = angleError * angleKP;
    
    // make sure turnSpeed is not greater than MaxTurnSpeed
    turnSpeed = ((Math.abs(turnSpeed) > MaxTurnSpeed ? Math.copySign(MaxTurnSpeed, angleError): turnSpeed));
    driveTrain.arcadeDrive(driveSpeed, turnSpeed);
    distanceError = endPosition-driveTrain.getPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(distanceError))<=tolerance);
  }
}
