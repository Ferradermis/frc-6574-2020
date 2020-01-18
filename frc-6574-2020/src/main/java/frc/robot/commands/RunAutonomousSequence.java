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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class RunAutonomousSequence extends InstantCommand {
  /**
   * Creates a new RunAutonomousSequence.
   */
  DriveTrain driveTrain;

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
    shoot();

    // Drive Backward 10 feet
    SmartDashboard.putString("Running Autonomous - Status", "Driving Backward(10)");
    Timer.delay(5.0);
    driveBackward(10);
  
    //Turn Left
    SmartDashboard.putString("Running Autonomous - Status", "Turning Left");
    Timer.delay(5.0);
    // turnLeft();
    
  }

  private void driveBackward(double distance){
    final double feetPerSecond = 2.5;
    double time = distance / feetPerSecond;  
    driveTrain.arcadeDrive(-.25, 0);
    Timer.delay(2.0);
    driveTrain.arcadeDrive(0,0);
   // CommandScheduler.getInstance().schedule((new DriveByTime(driveTrain, -.25, 0.0,time)));
    //.withTimeout(time));
    
  //  CommandScheduler.getInstance().schedule(new StartEndCommand(()->driveTrain.arcadeDrive(.25,0),
  //  ()->driveTrain.arcadeDrive(0,0), driveTrain)).withTimeout(2.0);
  
  // CommandScheduler.getInstance().schedule(new ArcadeDrive(driveTrain));
  }

  private void driveForward(double distance){
    final double feetPerSecond = 10.0;
    double time = distance / feetPerSecond;  
 //   CommandScheduler.getInstance().schedule((new DriveByTime(driveTrain, .25, 0.0)).withTimeout(time));
 //   CommandScheduler.getInstance().schedule(new ArcadeDrive(driveTrain));
  }

  private void shoot() {
    Robot.leds.set(.71);
    Timer.delay(5.0);
  }
}
