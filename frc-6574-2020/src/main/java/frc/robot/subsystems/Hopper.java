/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  final double MaxHopperSpeed = 0.25;

  public CANSparkMax hopperMotor = new CANSparkMax(RobotMap.HOPPER_CAN_ID, MotorType.kBrushless);


  
  public Hopper() {
    double rampRate = 0.2;
    int currentLimit = 30; 
 
    hopperMotor.setOpenLoopRampRate(rampRate);
    hopperMotor.setSmartCurrentLimit(currentLimit);
  }

  public void turnOn() {
    hopperMotor.set(MaxHopperSpeed);
  }

  public void turnOff() {
    hopperMotor.set(0);
  }

  public void reverseOn() {
    hopperMotor.set(-MaxHopperSpeed);
  }

  /*
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  */
}
