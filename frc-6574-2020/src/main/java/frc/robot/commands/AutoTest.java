/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;


public class AutoTest extends InstantCommand {

  DriveTrain driveTrain;
  
 
  public AutoTest(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HelperMethods.allAutoStart();
   
    driveTrain.turnToHeading(-45);
    driveTrain.driveAlongAngle(4, 1, -45); 
    driveTrain.turnToHeading(0.0); 
    
    HelperMethods.allAutoEnd();
  }
}