/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

  /*******************************************************************
  *  NOTE this drivetrain code assumes that the left motor gear boxes are installed in "reverse"
  * spinleft function will need to be tweaked if new gearboxes are installed opposite
  *
  *  NOTE if new controls for motors are installed need to fix:
  * Controllers are defined and instantiated as class variables
  * Controller setup is done in class function
  *******************************************************************/

public class DriveTrain extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  

  private CANSparkMax frontLeft = new CANSparkMax(RobotMap.FRONT_LEFT_CAN_ID, MotorType.kBrushless);
  private CANSparkMax backLeft = new CANSparkMax(RobotMap.BACK_LEFT_CAN_ID, MotorType.kBrushless);
  private CANSparkMax frontRight = new CANSparkMax(RobotMap.FRONT_RIGHT_CAN_ID, MotorType.kBrushless);
  private CANSparkMax backRight = new CANSparkMax(RobotMap.BACK_RIGHT_CAN_ID, MotorType.kBrushless);


  public DriveTrain(){
    double rampRate = 0.2; //time in seconds to go from 0 to full throttle; 0.2 is selected on feel by drivers for 2019
    int currentLimit = 30; //int because .setSmartCurrentLimit takes only ints, not doubles. Which makes sense programmatically. 

    frontLeft.setOpenLoopRampRate(rampRate);
    frontLeft.setSmartCurrentLimit(currentLimit);

    frontRight.setOpenLoopRampRate(rampRate);
    frontRight.setSmartCurrentLimit(currentLimit);

    backLeft.setOpenLoopRampRate(rampRate);
    backLeft.setSmartCurrentLimit(currentLimit);

    backRight.setOpenLoopRampRate(rampRate);
    backRight.setSmartCurrentLimit(currentLimit);
  }

  
  /**
   * Drives the robot using the arcade drive style,
   * @param drive is "speed" to move forward (positive) or backward (negative)
   * @param steer is "amount" to turn right (positive) or left (negative)
   * best to pass in normalized variables from 1 to -1 
   */
  public void arcadeDrive(double drive, double steer) {

    // if steer and drive are both too low, stop the motors and end
    if ((Math.abs(drive) <= 0.05) && (Math.abs(steer) <= 0.05)) {
      stop();
      return;
    }

    // if steer and drive are not too low, then calculate "speed" and move
   
    double leftSpeed = drive + steer;
    double rightSpeed = drive - steer;

    if (leftSpeed > 1) {
      leftSpeed = 1;
    } else if (leftSpeed < -1) {
      leftSpeed = -1;
    }

    if (rightSpeed  > 1) {
      rightSpeed = 1;
    } else if (rightSpeed < -1) {
      rightSpeed = -1;
    }

     spin(leftSpeed, rightSpeed);
  
    }
      
  private void spin(double leftSpeed, double rightSpeed){
    spinLeft(leftSpeed);
    spinRight(rightSpeed);
  }
  /**
   * Spins the two left side motors of the robot's drive base.
   * @param speed a double in the range of -1 to 1
   * 
   * Note current drivegears are faced "backwards" so need to invert the speed
   */
  private void spinLeft(double speed) {
    frontLeft.set(-speed);
    backLeft.set(-speed);
  }

  /**
   * Stops the two left side motors of the robot's drive base.
   */
  private void stopLeft() {
    spinLeft(0);
  }

  /**
   * Spins the two right side motors of the robot's drive base.
   * 
   * @param speed a double in the range of -1 to 1
   */
  private void spinRight(double speed) {
    frontRight.set(speed);
    backRight.set(speed);
  }

  /**
   * Stops the two right side motors of the robot's drive base.
   */
  private void stopRight() {
    spinRight(0);
  }

  /**
   * Stops the four motors of the robot's drive base.
   */
  private void stop() {
    stopLeft();
    stopRight();
  }

}