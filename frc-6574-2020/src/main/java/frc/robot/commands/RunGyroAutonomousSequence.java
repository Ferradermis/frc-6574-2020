/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class RunGyroAutonomousSequence extends InstantCommand {
  /**
   * Creates a new RunAutonomousSequence.
   */
  DriveTrain driveTrain;
  
  // TestPlan constants:  Use TestPlan to run simple tests
  final char TestPlan = 'T';
 
  // PlanA constants: Plan A starts in front of target, shoots 3 balls, retrieves first 3 balls in trench
  // need to decide if we want to pick up last two balls in trench
  // drive to target, shoot 3-5 balls
  final char PlanA = 'A';
  final double PlanAHeading1 = -40.0;
  final double PlanAHeading2 = -23.0;
  final double PlanASideA = 10.0;  //10.0
  final double PlanASideB = 7.0;  //7.0
  final double PlanASideC = 12.5; //12.5
 
  // PlanB constants:  Plan B starts in front of outside ball near opponents trench;
  // retrieve outside ball, retrieve inside ball, drive to target, shoot 5 balls
  final char PlanB = 'B';
  final double PlanBHeading1 = -45.0;
  final double PlanBHeading2 = 61.20;
  final double PlanBSideA = 10.83;
  final double PlanBSideB = 2.25;
  final double PlanBSideC = 3.25;
  final double PlanBSideD = 12.25; // 19.25
  //
  
  final double MaxDriveSpeed = 0.5;
  final double MaxTurnSpeed = 0.25;
  final double EncoderUnitsPerFeet = 14500;

  public RunGyroAutonomousSequence(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    char autonomousPlan = TestPlan;
    double startTime = Timer.getFPGATimestamp();
    System.out.println("Running Autonomous - Start Time:" + Timer.getFPGATimestamp());
    
    driveTrain.stop();
    driveTrain.resetGyro();

    if (autonomousPlan == TestPlan) {
      turnToHeading(20);
      Timer.delay(1);
      turnToHeading(60);
      Timer.delay(1);
      turnToHeading(30);
      Timer.delay(1);
      turnToHeading(-10);
      Timer.delay(1);
      turnToHeading(-70);
      Timer.delay(1);
      turnToHeading(30);

      driveAlongAngle(1, -1, 0);
    }
    else if (autonomousPlan == PlanA){
      // Shoot
      // RobotContainer.shooter.shoot(); 
   
      turnToHeading(PlanAHeading1);
      driveAlongAngle(PlanASideA,-1,PlanAHeading1); 
      turnToHeading(0.0); 
      //intakeOn(); 
      driveAlongAngle(PlanASideB, -1, 0.0); 
      // intakeOff(); 
      turnToHeading(PlanAHeading2); 
      driveAlongAngle(PlanASideC, 1, PlanAHeading2); 
      turnToHeading(0.0);
    //  driveTrain.stop();  // use this if we choose to remove stops with functions
      //aim(); 
      //robotContainer.shooter.shoot(); 
    }
    else if (autonomousPlan == PlanB)  // grabbing two opponents Power Cells near their 
    {
      // START NEAR OPPONENTS LOADING BAY, 
      // drive backward to get two power cells in opponent trench run
    driveAlongAngle(PlanBSideA, -1, 0.0);
    driveAlongAngle(PlanBSideB, 1, 0.0);
    turnToHeading(PlanBHeading1);
    driveAlongAngle(PlanBSideC, -1, PlanBHeading1);
    turnToHeading(PlanBHeading2);
    driveAlongAngle(PlanBSideD, 1, PlanBHeading2);
    turnToHeading(0.0);
    // aim
    // RobotContainer.shooter.shoot(); // should be shooting 5 power cells
    }
      
    double endTime = Timer.getFPGATimestamp();
    System.out.println("End Time:" + endTime);
    System.out.println("Run Time of Autonomous" + (endTime - startTime));
  }
  
 
  private void driveAlongAngle(double distanceInFeet, int direction, double alongAngle)
  {
    double driveSpeed = MaxDriveSpeed * direction;
    double distanceInEncoderUnits = direction * distanceInFeet * EncoderUnitsPerFeet;
    double distanceError = distanceInEncoderUnits; 
    
      
    double startPosition = driveTrain.getPosition();  
    double endPosition = startPosition + distanceInEncoderUnits;

    double angleError = alongAngle-driveTrain.getGyroAngle();
  
    if (Math.abs(angleError) > 1) {
      turnToHeading(alongAngle);
    }

    double currentPosition = driveTrain.getPosition();
    if (direction == -1){ // going backward
      while (currentPosition > endPosition){
        distanceError = endPosition-currentPosition;
        driveSpeed = (Math.abs(distanceError) > EncoderUnitsPerFeet) ? -MaxDriveSpeed :
             (Math.abs(distanceError) / EncoderUnitsPerFeet * -MaxDriveSpeed *.75 -.05);
        angleError = alongAngle-driveTrain.getGyroAngle();
        driveTrain.arcadeDrive(driveSpeed,angleError*.005);
        currentPosition = driveTrain.getPosition();
      }
    } else { // going forward
      while (currentPosition < endPosition) {
        distanceError = endPosition-currentPosition;
        driveSpeed = (Math.abs(distanceError) > ( EncoderUnitsPerFeet) ? MaxDriveSpeed :
             (Math.abs(distanceError) / EncoderUnitsPerFeet  * MaxDriveSpeed * .75 + .05));
        angleError = alongAngle-driveTrain.getGyroAngle();
        driveTrain.arcadeDrive(driveSpeed,angleError*.005);
        currentPosition = driveTrain.getPosition();
      }
    }
    driveTrain.stop();
  }

  private void turnToHeading(double intendedHeading) {    
    double turnSpeed = ((intendedHeading-driveTrain.getGyroAngle()) < 0 ? -MaxTurnSpeed : MaxTurnSpeed);
    double tolerance = 13;

    if (intendedHeading < driveTrain.getGyroAngle()){
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
