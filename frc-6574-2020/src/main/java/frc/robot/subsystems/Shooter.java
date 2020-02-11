/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // shooter/launcher is two falcons -- built in encoder talonFX
  private WPI_TalonFX launcher1 = new WPI_TalonFX(RobotMap.SPINNER1_CAN_ID);
  private WPI_TalonFX launcher2 = new WPI_TalonFX(RobotMap.SPINNER2_CAN_ID);

  // loader neo550's
  private CANSparkMax loader = new CANSparkMax(RobotMap.LOADER_CAN_ID, MotorType.kBrushless);
 
  // rotator vexPro775
  private TalonSRX turretRotator = new TalonSRX(RobotMap.TURRET_CAN_ID);
  
  // hood controller for raising and lowering
 // public DoubleSolenoid hoodController = new DoubleSolenoid(2, 3);
  // public Solenoid hoodController = new Solenoid(3);

  private double MAXROTATION = 45;

  Limelight limelight = new Limelight();

  private boolean shooting = false;

  public Shooter() {
    configureMotors();
    limelight.ledOn();
    limelight.setTarget(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((shooting == true)&&(limelight.hasTarget())) {
      spin(getDistanceToTarget());
      aim();
      if (aimed() && launcherReady(getDistanceToTarget())) {
          loadAndFire();
        } else { // shooting, but not aimed or not ready
          stopLoader();
        }
      } else { // not shooting or no target
        stopShooting(); // stops all motors
      }  
  }

  private void aim()
  {
    double kP = .01;

    // If no target in view; stop and exit
    if (limelight.hasTarget()) {
      double angleX = limelight.getAngleX();
      if (Math.abs(turretRotator.getSelectedSensorPosition())<MAXROTATION) {
        turretRotator.set(ControlMode.PercentOutput, angleX*kP);
      }  else {
        stopShooting();
      }
    } else {
      stopShooting();
    }
  }

  private double getDistanceToTarget() {
    //All calculations are in centimeters
    final double h2 = 86.36; //height of target
    final double h1 = 16.51; //height of camera
    // NOTE in final code, just calculate h2 - h1 and set a variable    
    final double A1 = 3.19; //Angle of camera relative to ground

    double angleY = limelight.getAngleY();
    
    // calculate currentDistance from target
    return (h2-h1)/Math.tan((angleY+A1)*Math.PI/180);
  }

  private boolean aimed() {       
    final double tolerance = 0.5;
    return (Math.abs(limelight.getAngleX()) < tolerance);
  }

  private void spin(double distance) {
    // need to figure out this formula to set velocity based on distance
    // see the html file linked at the top of this java file
    double targetVelocity_UnitsPer100ms = distance * 500.0 * 4096 / 600;
		/* 500 RPM in either direction */
		launcher1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  private boolean launcherReady(double distance) {
    // see if selected sensor velocity is within tolerance of the speed we want
    double something = .9999; // need to figure out what this should be
    double intendedVelocity = distance * something;
    return (launcher1.getSelectedSensorVelocity() >= intendedVelocity);
  }

  private void loadAndFire()
  {
    loader.set(.5);
  }

  public void shoot() {
    shooting = true;
  }

  public void stopShooting() {
    shooting = false;
    stopAiming();
    stopLauncher();
    stopLoader();
  }

  private void stopLauncher() {
    launcher1.set(ControlMode.PercentOutput, 0);
  }

  private void stopAiming() {
    turretRotator.set(ControlMode.PercentOutput, 0);
  }

  public void stopLoader()
  {
    loader.set(0);
  }
/*
  public void raiseHood()
  {
    hoodController.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerHood()
  {
    hoodController.set(DoubleSolenoid.Value.kReverse);
  }
*/
  private void configureMotors(){
    // Set up motors
    double rampRate = 0.2; //time in seconds to go from 0 to full throttle; 0.2 is selected on feel by drivers for 2019
 
    launcher2.follow(launcher1);
    launcher1.configFactoryDefault();
    launcher2.configFactoryDefault();
    launcher1.configOpenloopRamp(rampRate);
    launcher2.configOpenloopRamp(rampRate);
    launcher1.setNeutralMode(NeutralMode.Coast); // MAKE SURE WE ARE IN COAST MODE
    launcher2.setNeutralMode(NeutralMode.Coast); // MAKE SURE WE ARE IN COAST MODE

    // SEE CODE FROM: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java

    /* Config sensor used for Primary PID [Velocity] */
    launcher1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,
     0, Constants.kTimeoutMs);

    /**
    * Phase sensor accordingly. 
    * Positive Sensor Reading should match Green (blinking) Leds on Talon
    */
    launcher1.setSensorPhase(true);

    /* Config the peak and nominal outputs */
    launcher1.configNominalOutputForward(0, Constants.kTimeoutMs);
    launcher1.configNominalOutputReverse(0, Constants.kTimeoutMs);
    launcher1.configPeakOutputForward(1, Constants.kTimeoutMs);
    launcher1.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    double kF = 1023.0/7200.0; // ??
    double kP = 0.25;
    double kI = 0.001;
    double kD = 20;
    launcher1.config_kF(Constants.kPIDLoopIdx, kF, Constants.kTimeoutMs);
    launcher1.config_kP(Constants.kPIDLoopIdx, kP, Constants.kTimeoutMs);
    launcher1.config_kI(Constants.kPIDLoopIdx, kI, Constants.kTimeoutMs);
    launcher1.config_kD(Constants.kPIDLoopIdx, kD, Constants.kTimeoutMs);

    int currentLimit = 30; 

    loader.setOpenLoopRampRate(rampRate);
    loader.setSmartCurrentLimit(currentLimit);
  }
}