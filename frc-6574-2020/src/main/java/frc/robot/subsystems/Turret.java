/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */
  // rotator vexPro775
  private TalonSRX turretRotator = new TalonSRX(RobotMap.TURRET_CAN_ID);

 // private final AS5600EncoderPwm encoder = new AS5600EncoderPwm(turretRotator.getSensorCollection());

  public Limelight limelight = new Limelight();
  
  public Turret() {
    configureMotors();
    limelight.setTarget(0);
//    turretRotator.setSelectedSensorPosition(0); // need to think of best way to do this
  }

  @Override
  public void periodic() {
//    SmartDashboard.putNumber("Actual Turret Position: ", turretRotator.getSelectedSensorPosition());
  }

/*  public void resetTurretForward() {
    turretRotator.set(ControlMode.Position, 0);
  }
*/
  public void stopTurning() {
    turretRotator.set(ControlMode.PercentOutput, 0);
  }

  public void turnLeft() {
    turretRotator.set(ControlMode.PercentOutput, .25);
  }

  public void turnRight() {
    turretRotator.set(ControlMode.PercentOutput, .25);
  }

  public void turn(double speed){
    turretRotator.set(ControlMode.PercentOutput, speed);
  }

/*  public int currentDirection() {
    return turretRotator.getSelectedSensorPosition();
  }
*/

public void stopAiming()
{
  stopTurning();
  
  Command command = this.getCurrentCommand();
  if (command!=null) {
    if (command.getClass().equals(RobotContainer.aimTurret.getClass())) {
      command.cancel();
    }
  }
}

public void setTarget(String targetName) {
  int targetNumber = 0;
  if (targetName.equals("red")){
    targetNumber = 0;
  } else {
    targetNumber = 1;
  }
  limelight.setTarget(targetNumber);
}

  private void configureMotors(){
    // Set up motors
    // don't need rampRate?
    // should set currentLimit?

    double rampRate = 0.2; 
    int currentLimit = 10; 

    turretRotator.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    turretRotator.setInverted(true);

    turretRotator.configOpenloopRamp(rampRate);
    turretRotator.configContinuousCurrentLimit(currentLimit);
    turretRotator.enableCurrentLimit(true);
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

}