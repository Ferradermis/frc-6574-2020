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
  final double SideB = 5.0;  //5.0
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
    driveTrain.arcadeDrive(0, 0);
    driveTrain.resetGyro();
 //   driveTrain.resetPosition();
    double startTime = Timer.getFPGATimestamp();
    System.out.println("Running Autonomous - Start Time:" + Timer.getFPGATimestamp());

    // Shoot
    // RobotContainer.shooter.shoot(); //turns on black LED and delay .05 sec 
    turnToHeading(Heading1);
    driveAlongAngle(SideA,-1,Heading1); // drive backward 10 feet
    turnToHeading(0.0); 
    //intakeOn(); 
    driveAlongAngle(SideB, -1, 0.0); //Drive backwards to collect balls
   // intakeOff(); 
    turnToHeading(Heading2); //
    driveAlongAngle(SideC, 1, Heading2); //Drive forward to start position
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
    System.out.println("In dAA, endPosition actual = " + driveTrain.getPosition());

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
  //  driveTrain.resetPosition();
    SmartDashboard.putNumber("dFIEU start pos: ", driveTrain.getPosition());
  
    while (driveTrain.getPosition() < distanceinEU)
    {
      driveTrain.arcadeDrive(speed,0);
    }
    double endPos = driveTrain.getPosition();
    driveTrain.stop();
  
    SmartDashboard.putNumber("dFIEU end pos: ", endPos);
    Timer.delay(.05);                          // allow any remaining drift to occur
    SmartDashboard.putNumber("dFIEU drift: ", driveTrain.getPosition()-distanceinEU);
  }

  private void simpleDriveForward(double distanceInFeet) {
    //driveTrain.resetPosition();
    double distanceInEncoderUnits = distanceInFeet * EncoderUnitsPerFeet; 
 
    double startPosition = driveTrain.getPosition();
    double endPosition = startPosition + distanceInEncoderUnits;
    
    driveTrain.drivePositionControl(12000); 
  
    driveTrain.stop();
  }

  private void simpleDriveBackward(double distanceInFeet) {
    //driveTrain.resetPosition();
    double distanceInEncoderUnits = distanceInFeet * EncoderUnitsPerFeet; 
    
    double startPosition = driveTrain.getPosition();
    double endPosition = startPosition - distanceInEncoderUnits;

    driveTrain.drivePositionControl(-12000); 
  
    driveTrain.stop();
  }

  private void intakeOn() {
   }

  private void intakeOff() {
   }

   private void driveForward(double distance){
    driveAlongAngle(distance, 1, driveTrain.getGyroAngle());
  }

  private void driveBackward(double distance){
    driveAlongAngle(distance, -1, driveTrain.getGyroAngle());
  }
  private void drive(double distance, int direction) {
    driveAlongAngle(distance, direction, driveTrain.getGyroAngle());
  }


}
