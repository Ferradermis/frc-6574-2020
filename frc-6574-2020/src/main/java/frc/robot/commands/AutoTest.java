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


public class AutoTest extends InstantCommand {
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
  public AutoTest(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double startTime = Timer.getFPGATimestamp();
    System.out.println("Running Autonomous Plan ");
    System.out.println("Starting Time:" + Timer.getFPGATimestamp());
    
    driveTrain.stop();
    driveTrain.resetGyro();

   
      System.out.println("Starting at position: " +driveTrain.getPosition());
      driveTrain.turnToHeading(PlanAHeading1);
      driveTrain.driveAlongAngle(PlanASideA, 1, PlanAHeading1); 
      driveTrain.turnToHeading(0.0); 
    
    //  simpleDriveForward(3);
      System.out.println("Finishing at position: " +driveTrain.getPosition());


      
    
    
  //  driveTrain.stop();    // make sure we are stopped at end of autonomous
    double endTime = Timer.getFPGATimestamp();
    System.out.println("Ending Time:" + endTime);
    System.out.println("Run Time of Autonomous: " + (endTime - startTime));
  }

}
