/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Tracker extends SubsystemBase {
  /**
   * Creates a new Tracker.
   * 
   * Contains functions for 
   * 1) turning to heading
   * 2) driving a set distance while aimed at a specific heading
   * 
   * 
   * 
   */
  public Tracker() {

  }

  public void turnToHeading(double intendedHeading, double MaxTurnSpeed)
  {
    RobotContainer.leds.set(.65);

    double angleError = intendedHeading-RobotContainer.driveTrain.getGyroAngle();
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
  
      RobotContainer.driveTrain.arcadeDrive(0, (Math.abs(turnPower) > MaxTurnSpeed ? MaxTurnSpeed : turnPower));

      lastAngleError = angleError;
      angleError = intendedHeading-RobotContainer.driveTrain.getGyroAngle();
      sumAngleError = sumAngleError + angleError;
    }
    RobotContainer.driveTrain.arcadeDrive(0, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
