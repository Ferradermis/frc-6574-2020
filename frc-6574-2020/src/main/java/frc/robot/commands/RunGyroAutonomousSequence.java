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
   // System.out.println("Running Autonomous - Status: Shooting");
    // RobotContainer.shooter.shoot(); //turns on black LED and delay .05 sec 

    // Turn to Heading 1
    System.out.println("Running Autonomous - Status: Turning to Heading1.");
    System.out.println("Starting Gyro setting" +  driveTrain.getGyroAngle());
    System.out.println("Starting Encoder position" + driveTrain.getPosition());
    turnToHeading(Heading1);
    System.out.println("Ending Gyro setting: " + driveTrain.getGyroAngle());
    System.out.println("Ending Encoder position: " + driveTrain.getPosition());
    System.out.println("Running Autonomous - Status: Done Turning to Heading 1");
    System.out.println("");
 
    // Drive SideA  
    System.out.println("Running Autonomous - Status Driving SideA");
    System.out.println("Starting Gyro setting: " +  driveTrain.getGyroAngle());
    System.out.println("Starting Encoder position: " + driveTrain.getPosition());
    driveAlongAngle(SideA,-1,Heading1); // drive backward 10 feet
    //driveAlongAngle(SideA, 1, Heading1);
 //   simpleDriveBackward(SideA);
 //   simpleDriveForward(SideA);
    System.out.println("Ending Gyro setting: " + driveTrain.getGyroAngle());
    System.out.println("Ending Encoder position: " + driveTrain.getPosition());
    System.out.println("Running Autonomous - Status: Done Driving SideA");
    System.out.println("");
   
    System.out.println("");
    System.out.println("Running Autonomous - Status: Turning Right to 0.0");
    System.out.println("Starting Gyro setting: " +  driveTrain.getGyroAngle());
    System.out.println("Starting Encoder position: " + driveTrain.getPosition());
    turnToHeading(0.0); 
    System.out.println("Ending Gyro setting: " + driveTrain.getGyroAngle());
    System.out.println("Ending Encoder position: " + driveTrain.getPosition());
    System.out.println("Running Autonomous - Status: Done Turning Right to 0.0");
    System.out.println("");
    
  //  SmartDashboard.putString("Running Autonomous - Status", "Intake On");
    //intakeOn(); //turn on gold LED and delay 1 second
  
   // SmartDashboard.putString("Running Autonomous - Status", "Driving Backward " + SideB + " feet");
    driveAlongAngle(SideB, -1, 0.0); //Drive backwards to collect balls
  //  System.out.println("Driving backward to collect power cells");

  //  SmartDashboard.putString("Running Autonomous - Status", "Intake Off");
   // intakeOff(); //turn on gold LED and delay 1 second
  
  //  SmartDashboard.putString("Running Autonomous - Status", "Turning Left to " + Heading2);
    turnToHeading(Heading2); //
   // System.out.println("Turning to Heading 2");
  //  SmartDashboard.putString("Running Autonomous - Status", "Driving Forward " + SideC + " feet.");
    driveAlongAngle(SideC, 1, Heading2); //Drive forward to start position
  //  System.out.println("Driving forward to start position");
  
  //  SmartDashboard.putString("Running Autonomous - Status", "Done turning Right to 0.0");
    //driveAlongAngle(1,1,0); //turn to target
  //  System.out.println("Turning to target");
  
    turnToHeading(0.0);
    //driveTrain.arcadeDrive(0,0);

    //SmartDashboard.putString("Running Autonomous - Status", "Aiming");
    //aim(); //turns on dark green LED and delay 2 sec
  
    //SmartDashboard.putString("Running Autonomous - Status", "Shooting");
    //RobotContainer.shooter.shoot(); //turns on black LED and delay 3 sec  
    double endTime = Timer.getFPGATimestamp();
    System.out.println("End Time:" + endTime);
    System.out.println("Run Time of Autonomous" + (endTime - startTime));
  }
  
 
  private void driveAlongAngle(double distanceInFeet, int direction, double alongAngle)
  {
    double driveSpeed = MaxDriveSpeed * direction;
    double distanceInEncoderUnits = direction * distanceInFeet * EncoderUnitsPerFeet; 
    
  //  driveTrain.resetPosition();    
    
    double startPosition = driveTrain.getPosition();  
    double endPosition = startPosition + distanceInEncoderUnits;
    System.out.println("In dAA, startPosition = " + startPosition);
    System.out.println("In dAA, endPosition intended = " + endPosition);

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

//  @Override
//  public void end(boolean interrupted) {
//    CommandScheduler.getInstance().schedule(arcadeDrive);
//  }


}
