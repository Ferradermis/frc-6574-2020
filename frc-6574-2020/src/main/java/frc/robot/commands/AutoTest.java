/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoTest extends InstantCommand {

  DriveTrain driveTrain;
  
 
  public AutoTest(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
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
 
    double delay = SmartDashboard.getNumber("Delay", 0.0);
    if (delay > 0.0) {
      Timer.delay(delay);
    }
   
    System.out.println("Starting at position: " +driveTrain.getPosition());
    driveTrain.turnToHeading(-45);
    driveTrain.driveAlongAngle(4, 1, -45); 
    driveTrain.turnToHeading(0.0); 
    System.out.println("Finishing at position: " +driveTrain.getPosition());
    
    //  driveTrain.stop();    // make sure we are stopped at end of autonomous
    double endTime = Timer.getFPGATimestamp();
    System.out.println("Ending Time:" + endTime);
    System.out.println("Run Time of Autonomous: " + (endTime - startTime));
  }
}