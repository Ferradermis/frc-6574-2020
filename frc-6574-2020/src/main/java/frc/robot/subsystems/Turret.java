/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */

  // rotator vexPro775
  private TalonSRX turretRotator = new TalonSRX(RobotMap.TURRET_CAN_ID);

  private final AS5600EncoderPwm encoder = new AS5600EncoderPwm(turretRotator.getSensorCollection());
  
  private double MAXROTATION = 45;

  Limelight limelight = new Limelight();
  
  private boolean aiming = false;

  public Turret() {
    configureMotors();
    limelight.ledOn();
    limelight.setTarget(0);
    turretRotator.setSelectedSensorPosition(0); // need to think of best way to do this
  }

  @Override
  public void periodic() {
  }

  private void aim()
  {
    aiming = true;
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

  public double getDistanceToTarget() {
    //All calculations are in centimeters
    final double h2 = 86.36; //height of target
    final double h1 = 21; //height of camera
    // NOTE in final code, just calculate h2 - h1 and set a variable    
    final double A1 = 10; //Angle of camera relative to ground

    double angleY = limelight.getAngleY();
    
    // calculate currentDistance from target
    return (h2-h1)/Math.tan((angleY+A1)*Math.PI/180);
  }

  public boolean aimed() {       
    final double tolerance = 0.5;
    return (Math.abs(limelight.getAngleX()) < tolerance);
  }

  public void resetTurretForward() {
    turretRotator.set(ControlMode.Position, 0);
  }

  public void stopAiming() {
    turretRotator.set(ControlMode.PercentOutput, 0);
    aiming = false;
  }

/** * Reads PWM values from the AS5600. 
 * 
 * THIS DOES NOT WORK - digital encoder sends random values
 */
/*
public class AS5600EncoderPwm {    
  private final SensorCollection sensors;    
  private volatile int lastValue = Integer.MIN_VALUE;    
  public AS5600EncoderPwm(SensorCollection sensors) {        
    this.sensors = sensors;
  }    
  public int getPwmPosition() {
    int raw = sensors.getPulseWidthRiseToFallUs();
    if (raw == 0) {
      int lastValue = this.lastValue;
      if (lastValue == Integer.MIN_VALUE) {
        return 0;
      }
      return lastValue;
    }
    int actualValue = Math.min(4096, raw - 128);
    lastValue = actualValue;
    return actualValue;    
  }
}
*/
  public void testTurnTurret()
  {
  //  turretRotator.getSensorCollection().setPulseWidthPosition(0, 60);
    System.out.println("Turret Rotator sensor at start: " + turretRotator.getSelectedSensorPosition());
    System.out.println("Turret Rotator PWM  at start: " + turretRotator.getSensorCollection().getPulseWidthRiseToFallUs());
//    System.out.println("Turret Rotator using AS5800  at start: " + encoder.getPwmPosition());
    
    turretRotator.set(ControlMode.PercentOutput,-.35);
    Timer.delay(.5);
    turretRotator.set(ControlMode.PercentOutput,0);
    Timer.delay(.5);
    System.out.println("Turret Rotator sensor at end: " + turretRotator.getSelectedSensorPosition());
    System.out.println("Turret Rotator PWM  at end: " + turretRotator.getSensorCollection().getPulseWidthRiseToFallUs());
 //   System.out.println("Turret Rotator using AS5800  at end: " + encoder.getPwmPosition());

  }

  private void configureMotors(){
    // Set up motors
    // don't need rampRate?
    // should set currentLimit?

    double rampRate = 0.2; 
    int currentLimit = 35; 

    turretRotator.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
  }
}