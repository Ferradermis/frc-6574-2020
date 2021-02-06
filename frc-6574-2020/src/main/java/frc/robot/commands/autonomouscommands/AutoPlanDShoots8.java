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


public class AutoPlanDShoots8 extends InstantCommand {
  
  DriveTrain driveTrain;
  
  // PLAN D Shoots 3; picks up 5 balls from trench, drives forward and shoots.

  public AutoPlanDShoots8(DriveTrain driveTrain) {
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
    driveTrain.turnToHeading(0.0);
    //RobotContainer.intake.intakeOn(); 
    driveTrain.driveAlongAngle(16, 0); 
    //RobotContainer.intake.intakeOff(); 
    driveTrain.turnToHeading(-45); 
    driveTrain.driveAlongAngle(3, -27); 
    driveTrain.driveAlongAngle(-3, -27); 
    driveTrain.turnToHeading(45);
    driveTrain.driveAlongAngle(3, 27); 
    driveTrain.driveAlongAngle(-14, 0);
//    RobotContainer.shoot.schedule();    
    
    HelperMethods.allAutoEnd();
  }
}