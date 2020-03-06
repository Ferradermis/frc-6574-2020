/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  final double HopperSpeedShooting = -0.6;
  final double HopperSpeedIntake = -0.25;
  final double AgitatorSpeed = 0.25;

  public CANSparkMax hopperMotor = new CANSparkMax(RobotMap.HOPPER_CAN_ID, MotorType.kBrushless);
  public CANSparkMax agitatorMotor = new CANSparkMax(RobotMap.AGITATOR_CAN_ID, MotorType.kBrushless);
  
  public Hopper() {
    configureMotors();
  }

  public void turnOnForShooting() {
    hopperMotor.set(HopperSpeedShooting);
    agitatorMotor.set(-AgitatorSpeed);
  }

  public void turnOnForIntake() {
    hopperMotor.set(HopperSpeedIntake);
    agitatorMotor.set(AgitatorSpeed);
  }

  public void turnOff() {
    hopperMotor.set(0);
    agitatorMotor.set(0);
  }

/*  public void testAgitator(){
    agitatorMotor.set(RobotContainer.oi.getOperatorLeftTrigger());  
    SmartDashboard.putNumber("Agitator Speed Position: ", RobotContainer.oi.getOperatorLeftTrigger());
  }
*/

  public void reverseForIntake() {
    hopperMotor.set(-HopperSpeedIntake);
    agitatorMotor.set(-AgitatorSpeed/2);
  }

  private void configureMotors() {
    double rampRate = 0.2;
    int currentLimit = 30; 
 
    hopperMotor.setOpenLoopRampRate(rampRate);
    hopperMotor.setSmartCurrentLimit(currentLimit);
    agitatorMotor.setOpenLoopRampRate(rampRate);
    agitatorMotor.setSmartCurrentLimit(currentLimit);

    hopperMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    agitatorMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    agitatorMotor.setInverted(false);
  }
  /*
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
 */ 
}
