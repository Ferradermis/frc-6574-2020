/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;



public class DriveByTime extends CommandBase {
  /**
   * Creates a new DriveByTime.
   */
  DriveTrain driveTrain;
  double drive;
  double steer;
  double time;
  double startTime;
  Spark leds;

  public DriveByTime(DriveTrain driveTrain, double drive, double steer, double time) {
    this.driveTrain = driveTrain;
    this.drive = drive;
    this.steer = steer;
    this.time = time;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }
    
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
   SmartDashboard.putNumber("startTime", startTime);
  SmartDashboard.putNumber("Time", time);
  RobotContainer.leds.set(.61); // red for testing
  
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Drive in backward", drive);
    SmartDashboard.putNumber("Steer in backward", steer);
    
    driveTrain.arcadeDrive(drive, steer);
   // SmartDashboard.putNumber("startTime", startTime);
    //SmartDashboard.putNumber("currentTime", Timer.getFPGATimestamp());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
    // Robot.leds.set(.77);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return (Timer.getFPGATimestamp()>(startTime + time));
   // this.startTime
   //return false;
  }
}
