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

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  final double MaxIntakeSpeed = 0.25;

  public CANSparkMax intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
  public Solenoid intakeDeploy = new Solenoid(3);

  
  public Intake() {
    double rampRate = 0.2;
    int currentLimit = 30; 
 
    intakeMotor.setOpenLoopRampRate(rampRate);
    intakeMotor.setSmartCurrentLimit(currentLimit);
  }

  public void turnOn() {
    intakeMotor.set(MaxIntakeSpeed);
  }

  public void turnOff() {
    intakeMotor.set(0);
  }

  public void reverseOn() {
    intakeMotor.set(-MaxIntakeSpeed);
  }

  public void reverseOff() {
    intakeMotor.set(-MaxIntakeSpeed);
  }

  public void deployOrRetract() {
    intakeDeploy.set(!intakeDeploy.get());
  }

  public void deploy() {
    intakeDeploy.set(true);
  }

  public void retract() {
    intakeDeploy.set(false);
  }
  /*
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  */
}
