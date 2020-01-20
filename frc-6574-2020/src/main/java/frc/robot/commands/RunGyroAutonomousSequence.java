/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;


public class RunGyroAutonomousSequence extends InstantCommand {
  /**
   * Creates a new RunAutonomousSequence.
   */
  DriveTrain driveTrain;

  final double Heading1 = -35.0;
  final double Heading2 = -23.0;
  final double SideA = 12.2;
  final double SideB = 6.0;
  final double SideC = 17.5;
  final double MaxDriveSpeed = 0.5;
  final double MaxTurnSpeed = 0.5;
  final double FeetPerSecond = 5.0;

  public RunGyroAutonomousSequence(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveTrain.resetGyro();
    // Shoot
    SmartDashboard.putString("Running Autonomous - Status", "Shooting");
    RobotContainer.shooter.shoot(); //turns on black LED and delay 3 sec 

    // BEGIN TESTING
    SmartDashboard.putNumber("Starting Gyro setting", driveTrain.getGyroAngle());
    turnToHeading(90);
    SmartDashboard.putNumber("Ending Gyro setting", driveTrain.getGyroAngle());
    // END TESTING

  /*  SmartDashboard.putString("Running Autonomous - Status", "Turning Left");
    turnToHeading(Heading1); //turns on yellow LED and delay 2 sec (35 degrees)
  
    SmartDashboard.putString("Running Autonomous - Status", "Driving Backward(12.2)");
    drive(SideA, -1, Heading1); //Drive backwards 12.2 feet to first ball, turns on blue violet LED
  
    SmartDashboard.putString("Running Autonomous - Status", "Turning Right");
    turnToHeading(0.0); //turns on orange LED and delay 2 sec (35 degrees)
    
    SmartDashboard.putString("Running Autonomous - Status", "Intake On");
    //intakeOn(); //turn on gold LED and delay 1 second
  
    SmartDashboard.putString("Running Autonomous - Status", "Driving Backward(6)");
    drive(SideB, -1, 0.0); //Drive backwards 6 feet to collect balls, turns on blue violet LED

    SmartDashboard.putString("Running Autonomous - Status", "Intake On");
    //intakeOff(); //turn on gold LED and delay 1 second
  
    SmartDashboard.putString("Running Autonomous - Status", "Turning Left");
    turnToHeading(Heading2); //turns on yellow LED and delay 2 sec (23 degrees)
  
    SmartDashboard.putString("Running Autonomous - Status", "Driving Forward(17.5)");
    drive(SideC, 1, Heading2); //Drive forward 17.5 feet back to start, turns on aqua LED
  
    SmartDashboard.putString("Running Autonomous - Status", "Turning Right");
    turnToHeading(0.0); //turns on orange LED and delay 2 sec (23 degrees)
  
    SmartDashboard.putString("Running Autonomous - Status", "Aiming");
    //aim(); //turns on dark green LED and delay 2 sec
  
    SmartDashboard.putString("Running Autonomous - Status", "Shooting");
    RobotContainer.shooter.shoot(); //turns on black LED and delay 3 sec  
    */
  }

  // if this works, move to TRACKER
  private void drive(double distance){
    driveAlongAngle(distance, 1, driveTrain.getGyroAngle());
  }
  private void driveForward(double distance){
    driveAlongAngle(distance, 1, driveTrain.getGyroAngle());
  }

  private void driveBackward(double distance){
    driveAlongAngle(distance, -1, driveTrain.getGyroAngle());
  }
  private void drive(double distance, int direction) {
    driveAlongAngle(distance, direction, driveTrain.getGyroAngle());
  }

  private void driveAlongAngle(double distance, int direction, double alongAngle)
  {
    double driveSpeed = MaxDriveSpeed * direction;

    double angleError = alongAngle-driveTrain.getGyroAngle();
    final double angleKp = .05;
    final double angleKi = 0.0; // probably don't need
    final double angleKd = 0.0;
    final double timeKp = .05;
    final double timeKi = 0.0;  // should not need
    final double timeKd = 0.0;

    if (Math.abs(angleError) > 1) {
      turnToHeading(alongAngle);
    }

    double time = distance / FeetPerSecond; 
    double endTime = Timer.getFPGATimestamp() + time;

    double timeError = endTime-Timer.getFPGATimestamp();
    double sumTimeError = timeError;
    double lastTimeError = timeError;
    
    angleError = alongAngle-driveTrain.getGyroAngle();
    double sumAngleError = angleError;
    double lastAngleError = angleError;

    double drivePower;
    double turnPower;

    // time is a proxy for distance; will need to change to distance when sensors available
    while (timeError > 0){
      drivePower = (driveSpeed*timeError*timeKp)+(sumTimeError*timeKi) + ((timeError - lastTimeError)*timeKd);
      turnPower = ((angleError*angleKp*MaxTurnSpeed)+(sumAngleError*angleKi)+((lastAngleError-angleError) * angleKd));

      driveTrain.arcadeDrive(Math.abs(drivePower) > MaxDriveSpeed ? MaxDriveSpeed : drivePower,
            (Math.abs(turnPower) > MaxTurnSpeed ? MaxTurnSpeed : turnPower));

      lastTimeError = timeError;
      timeError = endTime-Timer.getFPGATimestamp();
      sumTimeError = sumTimeError + timeError;

      lastAngleError = angleError;
      angleError = alongAngle-driveTrain.getGyroAngle();
      sumAngleError = sumAngleError + angleError;
    }

    driveTrain.arcadeDrive(0,0);
  }

  private void turnToHeading(double intendedHeading) {
    RobotContainer.leds.set(.65);

    double angleError = intendedHeading-driveTrain.getGyroAngle();
    double sumAngleError = angleError;
    double lastAngleError = angleError;
    double turnPower;
    final double angleKp = .05;
    final double angleKi = 0.0;    // probably don't need Ki
    final double angleKd = 0.0;

    
    final double tolerance = .025; // set to angle error tolerance
                                  // should be able to decrease this the better PID control works

    while (Math.abs(angleError) > tolerance) {
      turnPower = ((angleError*angleKp*MaxTurnSpeed)+(sumAngleError*angleKi)+((lastAngleError-angleError) * angleKd));
  
      driveTrain.arcadeDrive(0, (Math.abs(turnPower) > MaxTurnSpeed ? MaxTurnSpeed : turnPower));

      lastAngleError = angleError;
      angleError = intendedHeading-driveTrain.getGyroAngle();
      sumAngleError = sumAngleError + angleError;
    }
    driveTrain.arcadeDrive(0, 0);
  }

  private void intakeOn() {
    RobotContainer.leds.set(.67);
    Timer.delay(1.0);
  }

private void intakeOff() {
    RobotContainer.leds.set(.67);
    Timer.delay(1.0);
  }

}
