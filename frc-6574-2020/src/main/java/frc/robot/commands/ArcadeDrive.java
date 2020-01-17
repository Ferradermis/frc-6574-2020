/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class ArcadeDrive extends CommandBase {

  private DriveTrain driveTrain;

  public ArcadeDrive(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
 public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  // NOTE  ALL Limelight code and buttonpressed tests should move to own commands

  @Override
  public void execute() {
    double steer_cmd=0;
    double drive_cmd=0;

    /* this code has all been moved to MoveToHighTarget command
    // IF xButton is pressed, then target the powerport
    if(RobotContainer.oi.l_xButton.get()){
      Limelight limelight = RobotContainer.limelight;

      //turns LED on
      limelight.ledOn();

      //Sets limelight to target powerport
      limelight.setTarget(0);

      // If no target in view; stop and exit
      if (limelight.hasTarget()) {
        driveTrain.arcadeDrive(0, 0);
        limelight.ledOff();
        return;
      }

      //All calculations are in centimeters
      final double h2 = 86.36; //height of target
      final double h1 = 16.51; //height of camera
      // NOTE in final code, just calculate h2 - h1 and set a variable
      
      final double DESIRED_DISTANCE = 150;  // how many cm's we want to be from target
      final double MAX_DRIVE = 0.25; // this can be changed to indicate maximum "speed"
      final double A1 = 3.19; //Angle of camera relative to ground

      double angleX = limelight.getAngleX();
      double angleY = limelight.getAngleY();
      SmartDashboard.putNumber("tx", angleX);
      SmartDashboard.putNumber("ty", angleY);
  
      
       // calculate currentDistance from target
       double currentDistance = (h2-h1)/Math.tan((angleY+A1)*Math.PI/180);

       // calculate "error" : how far we are from DESIRED DISTANCE
       double distanceError = (currentDistance - DESIRED_DISTANCE); 

       if (Math.abs(distanceError) < 5) // get within 2 inches of DESIRED_DISTANCE
       { 
         distanceError = 0; 
       }

       SmartDashboard.putNumber("currentDistance in Inches", currentDistance/2.54);

      steer_cmd = angleX/29.5; // normalize tx to -1 to 1
      drive_cmd = distanceError/1524; // normalize distanceError

      SmartDashboard.putNumber("steer cmd", steer_cmd);
      SmartDashboard.putNumber("drive cmd", drive_cmd);
  
      if  (drive_cmd > MAX_DRIVE) {
        driveTrain.arcadeDrive(MAX_DRIVE, steer_cmd);
      }
      else if (drive_cmd < -MAX_DRIVE) {
        driveTrain.arcadeDrive(-MAX_DRIVE, steer_cmd);
      }
      else {
      driveTrain.arcadeDrive(drive_cmd, steer_cmd);
      }

      limelight.ledOff();
      return;

    }
    */
    // IF bButton is pressed then target a ball and move to ball;
    if(RobotContainer.oi.l_bButton.get()){
       //sets pipeline
       Limelight limelight = RobotContainer.limelight;
       limelight.setTarget(1);
    
             
       if (!limelight.hasTarget()){
         driveTrain.arcadeDrive(0,0);
         return;
       }

       double angleX = limelight.getAngleX();
       SmartDashboard.putNumber("ball tx", angleX);

      // otherwise aim towards ball and move "forward"
      steer_cmd = angleX/27; // normalize tx
      drive_cmd = (.25); // drive forward at steady state
      SmartDashboard.putNumber("steer cmd", steer_cmd);
      SmartDashboard.putNumber("drive cmd", drive_cmd);
   
      driveTrain.arcadeDrive(drive_cmd, steer_cmd);
      return;
    }
    // Otherwise....
    //  NOTE THIS IS THE CODE THAT SHOULD REMAIN IN THIS FILE WHEN DONE TESTING
    // Read data from joystick and drive per joystick positioning
    double y = -RobotContainer.oi.driver.getRawAxis(1);
    double x = RobotContainer.oi.driver.getRawAxis(0);
  
    drive_cmd = Math.pow(y, 3); // cubing y makes it more "sensitive"
    steer_cmd = 0.5 * Math.pow(x, 3); // cubing x and /2 makes it more "sensitive"
  
    // dividing drive_cmd and steer_cmd by 8 during testing in classroom
    // delete division when on larger field and done testing
    driveTrain.arcadeDrive(drive_cmd/8, steer_cmd/8);   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}