/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;




public class DriveTrain extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private AHRS gyro = new AHRS(SerialPort.Port.kMXP); 
  private WPI_TalonFX frontLeft = new WPI_TalonFX(RobotMap.FRONT_LEFT_CAN_ID);
  private WPI_TalonFX backLeft = new WPI_TalonFX(RobotMap.BACK_LEFT_CAN_ID);
  private WPI_TalonFX frontRight = new WPI_TalonFX(RobotMap.FRONT_RIGHT_CAN_ID);
  private WPI_TalonFX backRight = new WPI_TalonFX(RobotMap.BACK_RIGHT_CAN_ID);

  public DriveTrain(){
    double kF = 0;
    double kP = 0.75;
    double kI = 0;
    double kD = 0;
    double rampRate = 0.2; //time in seconds to go from 0 to full throttle; 0.2 is selected on feel by drivers for 2019
    int currentLimit = 30; //int because .setSmartCurrentLimit takes only ints, not doubles. Which makes sense programmatically. 

    gyro.enableLogging(false);

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();
    backRight.configFactoryDefault();

    frontLeft.configOpenloopRamp(rampRate);
    backLeft.configOpenloopRamp(rampRate);
    frontRight.configOpenloopRamp(rampRate);
    backRight.configOpenloopRamp(rampRate);

//    frontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime));

    frontLeft.config_kP(0, kP);
    frontRight.config_kP(0, kP);
    backLeft.config_kP(0, kP);
    backRight.config_kP(0, kP);

    frontRight.setInverted(true);

    backRight.setInverted(true);
   
   //gyro.calibrate();
  }

  /**
   * Drives the robot using the arcade drive style,
   * @param drive is "speed" to move forward (positive) or backward (negative)
   * @param steer is "amount" to turn right (positive) or left (negative)
   * best to pass in normalized variables from 1 to -1 
   */
  public void arcadeDrive(double drive, double steer) {

   
   // SmartDashboard.putNumber("Encoder Position", frontLeft.getSelectedSensorPosition());

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
    frontLeft.set(ControlMode.PercentOutput,speed);
//    backLeft.set(ControlMode.PercentOutput,speed);
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
    frontRight.set(ControlMode.PercentOutput,speed);
//    backRight.set(ControlMode.PercentOutput,speed);
  }

  /**
   * Stops the two right side motors of the robot's drive base.
   */
  private void stopRight() {
    spinRight(0);
  }

  /**
   * Stops the four motors of the robot's drive base.
   *
   * */
  public void stop() {
    stopLeft();
    stopRight();
  }

  /**
	 * Gets the angle of drive train from its initial position.
	 * 
	 * @return	a double containing the drive train's current orientation
	 */
	public double getGyroAngle() {
		return gyro.getAngle();
	}
	
	/**
	 * Resets the drive train's gyroscope position to the zero value.
	 */
	public void resetGyro() {
		gyro.reset();
	}
  
  public void drivePositionControl(double endPosition)
  {
    frontLeft.set(ControlMode.Position, endPosition);
    frontRight.set(ControlMode.Position, endPosition);
//    backRight.set(ControlMode.Position, endPosition);
//    backLeft.set(ControlMode.Position, endPosition);
  }

  
	public double getPosition() {
      return ((frontLeft.getSelectedSensorPosition()+frontRight.getSelectedSensorPosition())/2); 
  }

  public void resetPosition() {
    frontLeft.setSelectedSensorPosition(0, 0, 50); 
    frontRight.setSelectedSensorPosition(0, 0, 50); 
  }


}
