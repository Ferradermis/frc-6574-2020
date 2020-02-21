/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class RunGyroAutonomousSequence extends InstantCommand {
  /**
   * Creates a new RunAutonomousSequence.
   */
  DriveTrain driveTrain;
  
  // TestPlan constants:  Use TestPlan to run simple tests
  final String TestPlan = "Test Plan";

  // PlanA constants: Plan A starts in front of target, shoots 3 balls, retrieves first 3 balls in trench
  // need to decide if we want to pick up last two balls in trench
  // drive to target, shoot 3-5 balls
  final String PlanA = "Plan A";
  final double PlanAHeading1 = -35.0;
  final double PlanAHeading2 = -23.0;
  final double PlanASideA = 8.0;  //10.0
  final double PlanASideB = 6.0;  //7.0
  final double PlanASideC = 15; //12.5
 
  // PlanB constants:  Plan B starts in front of outside ball near opponents trench;
  // retrieve outside ball, retrieve inside ball, drive to target, shoot 5 balls
  final String PlanB = "Plan B";
  final double PlanBHeading1 = -45.0;
  final double PlanBHeading2 = 61.20;
  final double PlanBSideA = 10.83;
  final double PlanBSideB = 2.25;
  final double PlanBSideC = 3.25;
  final double PlanBSideD = 12.25; // 19.25
  //
  
/*  final double MaxDriveSpeed = 0.5;
  final double MaxTurnSpeed = 0.25;
  final double EncoderUnitsPerFeet = 14500;
*/
  public RunGyroAutonomousSequence(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String autonomousPlan = "Test Plan"; //delete when new code works
    double startTime = Timer.getFPGATimestamp();
    System.out.println("Running Autonomous Plan " + autonomousPlan);
    System.out.println("Starting Time:" + Timer.getFPGATimestamp());
    
    driveTrain.stop();
    driveTrain.resetGyro();

    if (autonomousPlan.equals(TestPlan)) {  
      System.out.println("Starting at position: " +driveTrain.getPosition());
      driveTrain.turnToHeading(PlanAHeading1);
      driveTrain.driveAlongAngle(PlanASideA, 1, PlanAHeading1); 
      driveTrain.turnToHeading(0.0); 
    
    //  simpleDriveForward(3);
      System.out.println("Finishing at position: " +driveTrain.getPosition());


      // retest turnToHeading and driveAlongAngle to confirm they work
      //test simpleDriveForward(1); should driveForward 1 foot using position control
      //intakeOn();
//      turnToHeading(45);
//      System.out.println("Should be 45; " + driveTrain.getGyroAngle());
//      Timer.delay(2);
  //    turnToHeading(0);
  //    System.out.println("Should be 0; " + driveTrain.getGyroAngle());
  //    Timer.delay(2);
     //DriveForward(3);
  //    driveAlongAngle(3, 1, 0);
      //driveAlongAngle(1, 1, 0);
     // System.out.println("Should be 0; " + driveTrain.getGyroAngle());
      }
    else if (autonomousPlan.equals(PlanA)){
      // Shoot
      // RobotContainer.shooter.shoot(); 
   
      driveTrain.turnToHeading(PlanAHeading1);
      driveTrain.driveAlongAngle(PlanASideA, 1, PlanAHeading1); 
      driveTrain.turnToHeading(0.0); 
      //RobotContainer.intake.intakeOn(); 
      driveTrain.driveAlongAngle(PlanASideB, 1, 0.0); 
      //RobotContainer.intake.intakeOff(); 
      driveTrain.turnToHeading(PlanAHeading2); 
      driveTrain.driveAlongAngle(PlanASideC, -1, PlanAHeading2); 
      driveTrain.turnToHeading(0.0);
      //robotContainer.shooter.aim(); 
      //robotContainer.shooter.shoot(); 
    }
    else if (autonomousPlan.equals(PlanB))  // grabbing two opponents Power Cells near their 
    {
      // START NEAR OPPONENTS LOADING BAY, 
      // drive backward to get two power cells in opponent trench run
      driveTrain.driveAlongAngle(PlanBSideA, 1, 0.0);
      driveTrain.driveAlongAngle(PlanBSideB, -1, 0.0);
      driveTrain.turnToHeading(PlanBHeading1);
      driveTrain.driveAlongAngle(PlanBSideC, 1, PlanBHeading1);
      driveTrain.turnToHeading(PlanBHeading2);
      driveTrain.driveAlongAngle(PlanBSideD, -1, PlanBHeading2);
      driveTrain.turnToHeading(0.0);
    // RobotContainer.shooter.aim()
    // RobotContainer.shooter.shoot(); // should be shooting 5 power cells
    }
    
  //  driveTrain.stop();    // make sure we are stopped at end of autonomous
    double endTime = Timer.getFPGATimestamp();
    System.out.println("Ending Time:" + endTime);
    System.out.println("Run Time of Autonomous: " + (endTime - startTime));
  }
  
 
/* MOVED TO DRIVETRAIN  
private void driveAlongAngle(double distanceInFeet, int direction, double alongAngle)
  {
    double kF = 0.1;
    double kP = 0.75;
    double tolerance = 750; // this would be roughly 1 inch

    double angleKP = .005;
    
    double driveSpeed;
    double turnSpeed = 0.0;
    double distanceError = distanceInFeet * EncoderUnitsPerFeet * direction;    
    double endPosition = driveTrain.getPosition() + distanceError;

    double angleError = alongAngle-driveTrain.getGyroAngle();
    
   // this code can be uncommented if we want to make sure we turn to Heading first
   // if (Math.abs(angleError) > 1) {
   //   turnToHeading(alongAngle);
   // }

      while (Math.abs(distanceError) > tolerance){
        driveSpeed = distanceError / EncoderUnitsPerFeet / 5 * kP + Math.copySign(kF,distanceError);
        // make sure we go no faster than MaxDriveSpeed
        driveSpeed = ((Math.abs(driveSpeed) > MaxDriveSpeed) ? Math.copySign(MaxDriveSpeed, driveSpeed) :  driveSpeed);
        angleError = alongAngle-driveTrain.getGyroAngle();
        turnSpeed = angleError * angleKP;
        // make sure turnSpeed is not greater than MaxTurnSpeed
        turnSpeed = ((Math.abs(turnSpeed) > MaxTurnSpeed ? Math.copySign(MaxTurnSpeed, angleError): turnSpeed));
        driveTrain.arcadeDrive(driveSpeed, turnSpeed);
        distanceError = endPosition-driveTrain.getPosition();
      }
    
    driveTrain.stop();
  }
*/

/*  MOVED TO DRIVETRAIN
  private void turnToHeading(double intendedHeading) {  
    double kF = 0.05;
    double kP = 0.02; 
    double angleError;
    double turnSpeed;
    double tolerance = 3;

    angleError = intendedHeading - driveTrain.getGyroAngle();
    while (Math.abs(angleError) > tolerance) {    
        turnSpeed = angleError * kP + Math.copySign(kF, angleError);
        // make sure turnSpeed is not greater than MaxTurnSpeed
        turnSpeed = ((Math.abs(turnSpeed) > MaxTurnSpeed ? Math.copySign(MaxTurnSpeed, angleError): turnSpeed));
        driveTrain.arcadeDrive(0, turnSpeed);
        angleError = intendedHeading - driveTrain.getGyroAngle();
      }

    driveTrain.stop();
  }
*/
/* MOVED TO DRIVETRAIN
  private void simpleDriveForward(double distanceInFeet) {
    double distanceInEncoderUnits = distanceInFeet * EncoderUnitsPerFeet; 
    driveTrain.drivePositionControl(distanceInEncoderUnits);  
  }
/*
  private void simpleDriveBackward(double distanceInFeet) {
    double distanceInEncoderUnits = distanceInFeet * EncoderUnitsPerFeet; 
    driveTrain.drivePositionControl(-distanceInEncoderUnits); 
  }
*/
}
