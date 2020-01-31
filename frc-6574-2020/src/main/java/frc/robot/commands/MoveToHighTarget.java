/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveToHighTarget extends CommandBase {
  DriveTrain driveTrain;
  boolean finished = false;
  
  /**
   * Creates a new MoveToHighTarget.
   */
  public MoveToHighTarget(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double steer_cmd=0;
    double drive_cmd=0;

    Limelight limelight = RobotContainer.limelight;

    //turns LED on
    limelight.ledOn();

    //Sets limelight to target powerport
    limelight.setTarget(0);

    // If no target in view; stop and exit
    if (!limelight.hasTarget()) {
      driveTrain.arcadeDrive(0, 0);
      limelight.ledOff();
      finished = true;
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
    
    if ((Math.abs(angleX)<.05)&&(Math.abs(distanceError)<0.05)) {
      finished = true;
    }
    limelight.ledOff();
    return;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
  /*  this code is just here in case needed at some point
  /*    if(RobotContainer.oi.l_bButton.get()){
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
*/

