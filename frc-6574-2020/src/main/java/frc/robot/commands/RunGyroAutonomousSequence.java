/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


public class RunGyroAutonomousSequence extends InstantCommand {
  /**
   * Creates a new RunAutonomousSequence.
   */
  DriveTrain driveTrain;

  final double Heading1 = -40.0;
  final double Heading2 = -23.0;
  final double SideA = 10.0;  //10.0
  final double SideB = 7.0;  //7.0
  final double SideC = 12.5; //12.5
  final double MaxDriveSpeed = 0.5;
  final double MaxTurnSpeed = 0.25;
  final double EncoderUnitsPerFeet = 13000;

  public RunGyroAutonomousSequence(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double startTime = Timer.getFPGATimestamp();
    System.out.println("Running Autonomous - Start Time:" + Timer.getFPGATimestamp());
    
    driveTrain.stop();
    driveTrain.resetGyro();
    // Shoot
    // RobotContainer.shooter.shoot(); 
    turnToHeading(Heading1);
    driveAlongAngle(SideA,-1,Heading1); 
    turnToHeading(0.0); 
    //intakeOn(); 
    driveAlongAngle(SideB, -1, 0.0); 
   // intakeOff(); 
    turnToHeading(Heading2); 
    driveAlongAngle(SideC, 1, Heading2); 
    turnToHeading(0.0);
    //driveTrain.stop();  // use this if we choose to remove stops with functions
    //aim(); 
    //robotContainer.shooter.shoot(); 
    
    double endTime = Timer.getFPGATimestamp();
    System.out.println("End Time:" + endTime);
    System.out.println("Run Time of Autonomous" + (endTime - startTime));
  }
  
 
  private void driveAlongAngle(double distanceInFeet, int direction, double alongAngle)
  {
    double driveSpeed = MaxDriveSpeed * direction;
    double distanceInEncoderUnits = direction * distanceInFeet * EncoderUnitsPerFeet; 
      
    double startPosition = driveTrain.getPosition();  
    double endPosition = startPosition + distanceInEncoderUnits;

    double angleError = alongAngle-driveTrain.getGyroAngle();
    if (Math.abs(angleError) > 1) {
      turnToHeading(alongAngle);
    }

    if (direction == -1){ // going backward
      while (driveTrain.getPosition() > endPosition){
        angleError = alongAngle-driveTrain.getGyroAngle();
        driveTrain.arcadeDrive(driveSpeed,angleError*.005);
      }
    } else { // going forward
      while (driveTrain.getPosition() < endPosition) {
        angleError = alongAngle-driveTrain.getGyroAngle();
        driveTrain.arcadeDrive(driveSpeed,angleError*.005);
      }
    }
    driveTrain.stop();
  }

  private void turnToHeading(double intendedHeading) {    
    double turnSpeed = ((intendedHeading-driveTrain.getGyroAngle()) < 0 ? -MaxTurnSpeed : MaxTurnSpeed);
    double tolerance = 13;

    if (intendedHeading < 0){
      while (driveTrain.getGyroAngle()>(intendedHeading+tolerance)) {
        driveTrain.arcadeDrive(0, turnSpeed);
      }
    }
    else {
      while (driveTrain.getGyroAngle()<(intendedHeading-tolerance)) {
        driveTrain.arcadeDrive(0, turnSpeed);
      }
    }
    driveTrain.stop();
  }

  private void driveForwardInEncoderUnits(double distanceinEU, double speed)
  {  
    while (driveTrain.getPosition() < distanceinEU)
    {
      driveTrain.arcadeDrive(speed,0);
    }
    driveTrain.stop();
}

  private void simpleDriveForward(double distanceInFeet) {
    double distanceInEncoderUnits = distanceInFeet * EncoderUnitsPerFeet; 
 
    double startPosition = driveTrain.getPosition();
    double endPosition = startPosition + distanceInEncoderUnits;
    
    driveTrain.drivePositionControl(12000); 
  
    driveTrain.stop();
  }

  private void simpleDriveBackward(double distanceInFeet) {
    double distanceInEncoderUnits = distanceInFeet * EncoderUnitsPerFeet; 
    
    double startPosition = driveTrain.getPosition();
    double endPosition = startPosition - distanceInEncoderUnits;

    driveTrain.drivePositionControl(-12000); 
  
    driveTrain.stop();
  }

}