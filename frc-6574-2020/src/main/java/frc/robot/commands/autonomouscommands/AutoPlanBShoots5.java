/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.HelperMethods;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class AutoPlanBShoots5 extends InstantCommand {
  /**
   * Creates a new RunAutonomousSequence.
   */
  DriveTrain driveTrain;
  
  // PlanB constants:  Plan B starts in front of outside ball near opponents trench;
  // retrieve outside ball, retrieve inside ball, drive to target, shoot 5 balls

  final double PlanBHeading1 = -45.0;
  final double PlanBHeading2 = 61.20;
  final double PlanBSideA = 10.83;
  final double PlanBSideB = 2.25;
  final double PlanBSideC = 3.25;
  final double PlanBSideD = 12.25; // 19.25
  
  public AutoPlanBShoots5(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HelperMethods.allAutoStart();
//    (new TurnTurretAtStart(RobotContainer.turret)).schedule();

    // START NEAR OPPONENTS LOADING BAY, 
    // drive backward to get two power cells in opponent trench run
    driveTrain.driveAlongAngle(PlanBSideA, 1, 0.0);
    driveTrain.driveAlongAngle(PlanBSideB, -1, 0.0);
    driveTrain.turnToHeading(PlanBHeading1);
    driveTrain.driveAlongAngle(PlanBSideC, 1, PlanBHeading1);
    driveTrain.turnToHeading(PlanBHeading2);
    driveTrain.driveAlongAngle(PlanBSideD, -1, PlanBHeading2);
    
    driveTrain.turnToHeading(0.0);
//    RobotContainer.shoot.schedule();    

    HelperMethods.allAutoEnd();
  } 
}