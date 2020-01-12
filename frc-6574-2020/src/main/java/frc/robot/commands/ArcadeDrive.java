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

  private final DriveTrain driveTrain;

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
 
    y = Math.pow(y, 3);
    x = 0.5 * Math.pow(x, 3);

    //Just here for testing.
    final double STEER_K = 0.004;
    final double DRIVE_K = 0.3;
    final double DESIRED_TARGET_AREA = .7;
    final double MAX_DRIVE = 0.25;

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

  
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("ta", ta);

   /* if(tx>1.0)
{
  tx = tx-.05f;
}
else if (tx<1.0)
{
  tx = tx+.05f;
}
*/
    if(RobotContainer.oi.l_xButton.get()){

      if (tv < 1.0) {
        driveTrain.arcadeDrive(0, 0);
        return;
      }
  
      double steer_cmd = tx * STEER_K;
      double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
      SmartDashboard.putNumber("steer cmd", steer_cmd);
    SmartDashboard.putNumber("drive cmd", drive_cmd);
  
      if (drive_cmd > MAX_DRIVE) {
        driveTrain.arcadeDrive(MAX_DRIVE, steer_cmd);
      }
      else   driveTrain.arcadeDrive(drive_cmd, steer_cmd);
      
     // y= ty/24.85;//dividing by four to make the robot slower
     // y = y-(0.68410462);
     // x= tx/29.8;
      /*if (ty < 16.9f)
{
  y = -.8;
}
else if (ty > 17.1f)
{
  y = .8;
}
    }    */
  } else
      driveTrain.arcadeDrive(y/4, x);
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