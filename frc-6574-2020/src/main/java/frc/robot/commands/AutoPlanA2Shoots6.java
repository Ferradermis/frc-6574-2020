/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;


public class AutoPlanA2Shoots6 extends InstantCommand {
  
  DriveTrain driveTrain;
  
  // PlanA constants: Plan A starts in front of target, shoots 3 balls, retrieves first 3 balls in trench
  // need to decide if we want to pick up last two balls in trench
  // drive to target, shoot 3-5 balls
  final double PlanAHeading1 = -35.0;
  final double PlanAHeading2 = -23.0;
  final double PlanASideA = 8.0;  //10.0
  final double PlanASideB = 6.0;  //7.0
  final double PlanASideC = 15; //12.5


  public AutoPlanA2Shoots6(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HelperMethods.allAutoStart();
//    (new TurnTurretAtStart(RobotContainer.turret)).schedule();

    // Shoot
    //    RobotContainer.shoot.schedule();  
    driveTrain.driveAlongAngle(3, 0.0);  
    driveTrain.turnToHeading(-60);
    RobotContainer.intake.deploy();
    driveTrain.driveAlongAngle(6, -60); 
    driveTrain.turnToHeading(0.0);  
    driveTrain.driveAlongAngle(6, 0.0); 
    //RobotContainer.intake.intakeOff(); 
    driveTrain.turnToHeading(-23); 
    driveTrain.driveAlongAngle(-14, -23); 
    driveTrain.turnToHeading(0.0);
//    RobotContainer.shoot.schedule();    
    
    HelperMethods.allAutoEnd();
  }
}