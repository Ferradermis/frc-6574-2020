/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  final double MaxIntakeSpeed = .875;
  final DoubleSolenoid.Value DEPLOYED = DoubleSolenoid.Value.kForward;
  final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kReverse;

  public CANSparkMax intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
  public DoubleSolenoid intakeDeploy = new DoubleSolenoid(RobotMap.INTAKE_EXTENDER_ID2, RobotMap.INTAKE_EXTENDER_ID1);

  
  public Intake() {
    configureMotors();
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Intake Motor Output Current", intakeMotor.getOutputCurrent());
  }

  public void turnOn() {
    intakeMotor.set(-MaxIntakeSpeed);
  }

  public void turnOff() {
    intakeMotor.set(0);
  }

  public void reverseOn() {
    intakeMotor.set(MaxIntakeSpeed);
  }

  public void deployOrRetract() {
    DoubleSolenoid.Value currentState = intakeDeploy.get();
    if (currentState == DEPLOYED) {
      retract();
    } else {
      deploy();
    }
  }

  public void deploy() {
    intakeDeploy.set(DEPLOYED);
    turnOn();  // TURNED OFF FOR TESTING
    RobotContainer.hopper.turnOnForIntake();
  }

  public void retract() {
    RobotContainer.hopper.turnOnForIntake();
    turnOff();
    intakeDeploy.set(RETRACTED);
    RobotContainer.hopper.turnOff();
  }

  private void configureMotors() {
    double rampRate = 0.2;
    int currentLimit = 20; 
 
    intakeMotor.setOpenLoopRampRate(rampRate);
    intakeMotor.setSmartCurrentLimit(currentLimit);
  }
  
}
