/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class RunAutonomousSequence extends InstantCommand {
  /**
   * Creates a new RunAutonomousSequence.
   */
  DriveTrain driveTrain;

final double angle1 = 35.0;
final double angle2 = 23.0;
final double sideA = 12.2;
final double sideB = 6.0;
final double sideC = 17.5;
final double driveSpeed = 0.5;
final double turnSpeed = 0.5;
final double feetPerSecond = 5.0;
final double degreesPerSecond = 90.0;

  public RunAutonomousSequence(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Shoot
    SmartDashboard.putString("Running Autonomous - Status", "Shooting");
    RobotContainer.shooter.shoot(); 

    SmartDashboard.putString("Running Autonomous - Status", "Turning Left");
    turnLeft(90);
    
  /*  turnLeft(angle1); //turns on yellow LED and delay 2 sec (35 degrees)
  
    SmartDashboard.putString("Running Autonomous - Status", "Driving Backward(12.2)");
    driveBackward(sideA); //Drive backwards 12.2 feet to first ball, turns on blue violet LED
  
    SmartDashboard.putString("Running Autonomous - Status", "Turning Right");
    turnRight(angle1); //turns on orange LED and delay 2 sec (35 degrees)
    
    SmartDashboard.putString("Running Autonomous - Status", "Intake On");
    //intakeOn(); //turn on gold LED and delay 1 second
  
    SmartDashboard.putString("Running Autonomous - Status", "Driving Backward(6)");
    driveBackward(sideB); //Drive backwards 6 feet to collect balls, turns on blue violet LED

    SmartDashboard.putString("Running Autonomous - Status", "Intake On");
    //intakeOff(); //turn on gold LED and delay 1 second
  
    SmartDashboard.putString("Running Autonomous - Status", "Turning Left");
    turnLeft(angle2); //turns on yellow LED and delay 2 sec (23 degrees)
  
    SmartDashboard.putString("Running Autonomous - Status", "Driving Forward(17.5)");
    driveForward(sideC); //Drive forward 17.5 feet back to start, turns on aqua LED
  
    SmartDashboard.putString("Running Autonomous - Status", "Turning Right");
    turnRight(angle2); //turns on orange LED and delay 2 sec (23 degrees)
  
    SmartDashboard.putString("Running Autonomous - Status", "Aiming");
    //aim(); //turns on dark green LED and delay 2 sec
  
    SmartDashboard.putString("Running Autonomous - Status", "Shooting");
    RobotContainer.shooter.shoot(); //turns on black LED and delay 3 sec  
    */
  }

  private void driveBackward(double distance){
    RobotContainer.leds.set(.89);
    double time = distance / feetPerSecond;  
    driveTrain.arcadeDrive(-driveSpeed, 0);
    Timer.delay(time);
    driveTrain.arcadeDrive(0,0);
   // CommandScheduler.getInstance().schedule((new DriveByTime(driveTrain, -.25, 0.0,time)));
    //.withTimeout(time));
    
  //  then try the following... not sure what the implications are relative to driveTrain 
  //  new StartEndCommand(()->driveTrain.arcadeDrive(-driveSpeed,0), 
  //          ()->driveTrain.arcadeDrive(0,0)).withTimeout(time).schedule();
}

  private void driveForward(double distance){
    RobotContainer.leds.set(.81);
    double timeDelay = distance / feetPerSecond;  
    driveTrain.arcadeDrive(driveSpeed, 0);
    Timer.delay(timeDelay);
    driveTrain.arcadeDrive(0,0);
 //   CommandScheduler.getInstance().schedule((new DriveByTime(driveTrain, .25, 0.0)).withTimeout(time));
 //   CommandScheduler.getInstance().schedule(new ArcadeDrive(driveTrain));
  }

  private void turnRight(double degree) {
    RobotContainer.leds.set(.65);
    double timeDelay = degree / degreesPerSecond;
    driveTrain.arcadeDrive(0,turnSpeed);
    Timer.delay(timeDelay);
  }
  
  private void turnLeft(double degree) {
    RobotContainer.leds.set(.69);
    double timeDelay = degree / degreesPerSecond;
    driveTrain.arcadeDrive(0,-turnSpeed);
    Timer.delay(timeDelay);
    driveTrain.arcadeDrive(0,0);

  //  then try the following... not sure what the implications are relative to driveTrain 
  //  new StartEndCommand(()->driveTrain.arcadeDrive(0,-turnSpeed), 
  //          ()->driveTrain.arcadeDrive(0,0)).withTimeout(timeDelay).schedule();;
  }

  private void intakeOn() {
    RobotContainer.leds.set(.67);
    Timer.delay(1.0);
  }

private void intakeOff() {
    RobotContainer.leds.set(.67);
    Timer.delay(1.0);
  }

} // end class
