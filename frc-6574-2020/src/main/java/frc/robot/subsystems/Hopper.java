/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  final double HopperSpeedShooting = 0.1875; //.125 for IAC, .6 for PPT or normal use
  final double HopperSpeedManualIntake = 0.5; //~5700 units/sec
  final double HopperSpeedIntake = .5;
  final double closeLoopSpeed = 3000;

  public CANSparkMax hopperMotor = new CANSparkMax(RobotMap.HOPPER_CAN_ID, MotorType.kBrushless);
  public CANPIDController hopperPIDController;
  public CANEncoder hopperEncoder;

  public Hopper() {
    configureMotors();
    hopperPIDController = hopperMotor.getPIDController();
    hopperEncoder = hopperMotor.getEncoder();

    double kP = .0001025;
    double kI = 0;
    double kD = .85;
    double kF = .000092;

    double kIz = 0;
    double kMaxOutput = 1;
    double kMinOutput = -1;
    //double maxRPM = 15000;

    hopperPIDController.setP(kP);
    hopperPIDController.setI(kI);
    hopperPIDController.setD(kD);
    hopperPIDController.setFF(kF);
    hopperPIDController.setIZone(kIz);
    hopperPIDController.setOutputRange(kMinOutput, kMaxOutput);

  }

  public void closeLoopTurnOnForShooting() {
    hopperPIDController.setReference(5700, ControlType.kVelocity);
  }

  public void closeLoopTurnOnForIntake() {
    hopperPIDController.setReference(5700, ControlType.kVelocity);
  }

  public void turnOnForShooting() {
    hopperMotor.set(HopperSpeedShooting);
  }

  
    public void turnOnForIntake() {
        hopperMotor.set(HopperSpeedIntake);
  }
  
  public void turnOnForIntakeManual() {
    hopperMotor.set(HopperSpeedManualIntake);
  }

  public void turnOff() {
    hopperMotor.set(0);
  }
  public void reverseForIntake() {
    hopperMotor.set(-HopperSpeedManualIntake);
  }



  private void configureMotors() {
    double rampRate = 0.2;
    int currentLimit = 10; 
 
    hopperMotor.setOpenLoopRampRate(rampRate);
    hopperMotor.setSmartCurrentLimit(currentLimit);
    

    hopperMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Spindexer Velocity", hopperEncoder.getVelocity());
    SmartDashboard.putNumber("Spindexer Position", hopperEncoder.getPosition());
  }
  
}
