/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class DriveTrain extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private AHRS gyro = new AHRS(SerialPort.Port.kMXP); 
  private WPI_TalonFX frontLeft = new WPI_TalonFX(RobotMap.FRONT_LEFT_CAN_ID);
  private WPI_TalonFX backLeft = new WPI_TalonFX(RobotMap.BACK_LEFT_CAN_ID);
  private WPI_TalonFX frontRight = new WPI_TalonFX(RobotMap.FRONT_RIGHT_CAN_ID);
  private WPI_TalonFX backRight = new WPI_TalonFX(RobotMap.BACK_RIGHT_CAN_ID);
  //private WPI_TalonFX thirdRight = new WPI_TalonFX(RobotMap.THIRD_RIGHT_CAN_ID);
  //private WPI_TalonFX thirdLeft = new WPI_TalonFX(RobotMap.THIRD_LEFT_CAN_ID);

  // following variable are used in turnToHeading and driveAlongAngle
  final double MaxDriveSpeed = 0.45;
  final double MaxTurnSpeed = 0.25;
  final double EncoderUnitsPerFeet = 14500;

  public DriveTrain(){
    configureMotors();
    resetPosition();
   //gyro.calibrate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actual Gyro Heading: ", gyro.getAngle());
    SmartDashboard.putNumber("Acual Drive Position: ", getPosition());
  }

  /**
   * Drives the robot using the arcade drive style,
   * @param drive is "speed" to move forward (positive) or backward (negative)
   * @param steer is "amount" to turn right (positive) or left (negative)
   * best to pass in normalized variables from 1 to -1 
   */
  public void arcadeDrive(double drive, double steer) {
    // if steer and drive are both too low, stop the motors and end
    if ((Math.abs(drive) <= 0.00) && (Math.abs(steer) <= 0.00)) {
      stop();
      return;
    }

    double leftSpeed = drive + steer;
    double rightSpeed = drive - steer;

    if (leftSpeed > 1) { leftSpeed = 1; }
      else if (leftSpeed < -1) {leftSpeed = -1;}

    if (rightSpeed  > 1) {rightSpeed = 1;}
     else if (rightSpeed < -1) {rightSpeed = -1;}
   
     frontLeft.set(ControlMode.PercentOutput,-leftSpeed);
     frontRight.set(ControlMode.PercentOutput,-rightSpeed);
  }

    /**
	  * Stops all drivetrain wheels.
	  */
  public void stop() {
    frontLeft.set(ControlMode.PercentOutput,0);
    frontRight.set(ControlMode.PercentOutput,0);
  }

  //functions to support 
  public void driveAlongAngle(double distance, double alongAngle)
  {
    int direction = (distance > 0) ? 1 : -1;
    driveAlongAngle(Math.abs(distance), direction, alongAngle);
  }

  public void driveAlongAngle(double distanceInFeet, int direction, double alongAngle)
  {
    double kF = 0.1;  //kF is essentially minimal amount to drive
    double kP = 0.75;
    double tolerance = 750; // this would be roughly 1 inch

    double angleKP = .005;
    
    double driveSpeed;
    double turnSpeed = 0.0;
    double distanceError = distanceInFeet * EncoderUnitsPerFeet * direction;    
    double endPosition = getPosition() + distanceError;

    double angleError = alongAngle-getGyroAngle();
    
   // this code can be uncommented if we want to make sure we turn to Heading first
   // if (Math.abs(angleError) > 1) {
   //   turnToHeading(alongAngle);
   // }

      while (Math.abs(distanceError) > tolerance){
        driveSpeed = distanceError / EncoderUnitsPerFeet / 5 * kP + Math.copySign(kF,distanceError);
        // make sure we go no faster than MaxDriveSpeed
        driveSpeed = ((Math.abs(driveSpeed) > MaxDriveSpeed) ? Math.copySign(MaxDriveSpeed, driveSpeed) :  driveSpeed);
        angleError = alongAngle-getGyroAngle();
        turnSpeed = angleError * angleKP;
        // make sure turnSpeed is not greater than MaxTurnSpeed
        turnSpeed = ((Math.abs(turnSpeed) > MaxTurnSpeed ? Math.copySign(MaxTurnSpeed, angleError): turnSpeed));
        arcadeDrive(driveSpeed, turnSpeed);
        distanceError = endPosition-getPosition();
      }
    
    stop();
  }

  public void turnToHeading(double intendedHeading) {  
    double kF = 0.05;
    double kP = 0.02; 
    double angleError;
    double turnSpeed;
    double tolerance = 3;

    angleError = intendedHeading - getGyroAngle();
    while (Math.abs(angleError) > tolerance) {    
        turnSpeed = angleError * kP + Math.copySign(kF, angleError);
        // make sure turnSpeed is not greater than MaxTurnSpeed
        turnSpeed = ((Math.abs(turnSpeed) > MaxTurnSpeed ? Math.copySign(MaxTurnSpeed, angleError): turnSpeed));
        arcadeDrive(0, turnSpeed);
        angleError = intendedHeading - getGyroAngle();
      }

    stop();
  }


  /**
	 * Gets the angle of drive train from its initial position.
	 * @return	a double containing the drive train's current heading
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
  
  /**
	 * Gets the current position of the drive train 
	 * @return	a double containing the drive train's current position;
   *                        as an average of left and right position.
	 */
	public double getPosition() {
      return ((frontLeft.getSelectedSensorPosition()+frontRight.getSelectedSensorPosition())/2); 
  }

  // NOTE THIS FUNCTION CALL IS NON-BLOCKING; TRY TO AVOID USING
  private void resetPosition() {
    frontLeft.setSelectedSensorPosition(0, 0, 50); 
    frontRight.setSelectedSensorPosition(0, 0, 50); 
  }

  
/*  POSITION CONTROL DRIVING UNUSED CURRENTLY
  public void drivePositionControl(double distanceInEncoderValues)
  {
    System.out.println("Starting at left position: " +frontLeft.getSelectedSensorPosition());
    System.out.println("Starting at right position: " +frontRight.getSelectedSensorPosition());
    frontLeft.set(ControlMode.Position, frontLeft.getSelectedSensorPosition()+distanceInEncoderValues);
    frontRight.set(ControlMode.Position, frontRight.getSelectedSensorPosition()+distanceInEncoderValues);
//    Timer.delay(2);
//    When we take out this timer delay, we get odd behavior.  Won't work, one motor will move and the other 
//  won't, etc.
    System.out.println("Ending at left position: " +frontLeft.getSelectedSensorPosition());
    System.out.println("Ending at right position: " +frontRight.getSelectedSensorPosition());
  }
*/

  /*  public void simpleDriveForward(double distanceInFeet) {
    double distanceInEncoderUnits = distanceInFeet * EncoderUnitsPerFeet; 
    drivePositionControl(distanceInEncoderUnits);  
  }
*/

  private void configureMotors() {

    double rampRate = 0.35; //time in seconds to go from 0 to full throttle

    gyro.enableLogging(false);

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    //thirdRight.follow(frontRight);
    //thirdLeft.follow(frontRight);

    frontLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();
    backRight.configFactoryDefault();
    //thirdRight.configFactoryDefault();
    //thirdLeft.configFactoryDefault();

    frontLeft.configOpenloopRamp(rampRate);
    backLeft.configOpenloopRamp(rampRate);
    frontRight.configOpenloopRamp(rampRate);
    backRight.configOpenloopRamp(rampRate);
    //thirdLeft.configOpenloopRamp(rampRate);
    //thirdRight.configOpenloopRamp(rampRate);

    frontRight.setInverted(true);
    backRight.setInverted(true);
    //thirdRight.setInverted(true);


    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);

// no current limit set on drivetrain    
// int currentLimit = 30; //int because .setSmartCurrentLimit takes only ints, not doubles. Which makes sense programmatically. 
//    frontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerThresholdCurrent, triggerThresholdTime));

/*   Use if we start to do drive by POSITION Closed Loop
   double kF = 0;
    double kP = 0.05;
    double kI = 0;
    double kD = 0;
    frontLeft.config_kP(0, kP);
    frontRight.config_kP(0, kP);
    backLeft.config_kP(0, kP);
    backRight.config_kP(0, kP);
    thirdLeft.config_kP(0, kP);

  //  frontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
  //  frontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
  */

  }
}