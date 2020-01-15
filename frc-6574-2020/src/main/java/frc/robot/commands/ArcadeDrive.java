/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

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
  @Override
  public void execute() {
    double y = -RobotContainer.oi.driver.getRawAxis(1);
    double x = RobotContainer.oi.driver.getRawAxis(0);
    double steer_cmd=0;
    double drive_cmd=0;
  
 
    y = Math.pow(y, 3);
    x = 0.5 * Math.pow(x, 3);

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
     

      //turns LimeLight LED off
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
      

      SmartDashboard.putNumber("tv", tv);
      SmartDashboard.putNumber("tx", tx);
      SmartDashboard.putNumber("ty", ty);
      SmartDashboard.putNumber("ta", ta);

      final double STEER_K = 0.008;
      final double DRIVE_K = 0.3;
      
     
      SmartDashboard.putNumber("steer cmd", steer_cmd);
      SmartDashboard.putNumber("drive cmd", drive_cmd);

    if(RobotContainer.oi.l_xButton.get()){
      //turns LED on
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      //Sets pipeline
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

       tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
       tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
       ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
       ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

       //Just here for testing.
       
       final double h2 = 86.36; //height of target
       final double h1 = 16.51; //height of camera
       //double DESIRED_DISTANCE = 274.32‬‬; //desired distance of camera to target
       final double DESIRED_DISTANCE = 150;
       //final double DESIRED_TARGET_AREA = .7;
       final double MAX_DRIVE = 0.25;
        double currentDistance;
       final double A1 = 3.19; //changed 3.2

       currentDistance = (h2-h1)/Math.tan((ty+A1)*Math.PI/180);
       double distance = (currentDistance - DESIRED_DISTANCE);
       if (Math.abs(distance) < 5) distance = 0;
       SmartDashboard.putNumber("currentDistance", currentDistance/2.54);
      steer_cmd = tx * STEER_K;
      drive_cmd = (distance) * DRIVE_K; 

      //Turns LED on
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      if (tv < 1.0) {
        driveTrain.arcadeDrive(0, 0);
        return;
      }
      if  (drive_cmd > MAX_DRIVE) {
        driveTrain.arcadeDrive(MAX_DRIVE, steer_cmd);
        return;
      }
      if (drive_cmd < -MAX_DRIVE) {
        driveTrain.arcadeDrive(-MAX_DRIVE, steer_cmd);
        return;
      }
        driveTrain.arcadeDrive(drive_cmd, steer_cmd);
    

    } else if(RobotContainer.oi.l_bButton.get()){
       //sets pipeline
       NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
       tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); 
       tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
       steer_cmd = tx * STEER_K;
       drive_cmd = (.25);

      if (tv < 1.0) {
        driveTrain.arcadeDrive(0, 0);
        return;
      } 

      driveTrain.arcadeDrive(drive_cmd, steer_cmd);
    }
    
    else
      driveTrain.arcadeDrive(y/8, x);
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