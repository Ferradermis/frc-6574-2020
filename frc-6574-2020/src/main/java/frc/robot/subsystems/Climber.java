/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  final double ClimberSpeed = 0.50;

  public WPI_TalonFX leftClimb = new WPI_TalonFX(RobotMap.LEFT_CLIMB_CAN_ID);
  public WPI_TalonFX rightClimb = new WPI_TalonFX(RobotMap.RIGHT_CLIMB_CAN_ID);

  //public DoubleSolenoid climberDeploy = new DoubleSolenoid(RobotMap.CLIMBER_EXTENDER_ID2, RobotMap.CLIMBER_EXTENDER_ID1);

  final DoubleSolenoid.Value DEPLOYED = DoubleSolenoid.Value.kForward;
  final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kReverse;
  
  final double elevatorSpeed = .25;
  double climbHeightExtension = 125000; //MAX RECORDED NUMBER = 148589
  double climbHeightRetraction = 20000;

  public Climber() {
    double rampRate = 0.2;
    int currentLimit = 60; 
    int currentLimitThreshold = 75;
    double currentLimitThresholdTime = 1.0;
    double allowableCloseLoopError = 150;

    resetClimber();
    leftClimb.configAllowableClosedloopError(0, allowableCloseLoopError, 30);
    
    //rightClimb.follow(leftClimb);

    leftClimb.setInverted(true);
    rightClimb.setInverted(false);
    
    rightClimb.setNeutralMode(NeutralMode.Brake);
    leftClimb.setNeutralMode(NeutralMode.Brake);

    rightClimb.configOpenloopRamp(rampRate);
    leftClimb.configOpenloopRamp(rampRate);

    leftClimb.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
    rightClimb.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));

    leftClimb.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
    rightClimb.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));

    double leftkF = 0.0006; 
    double leftkP = .17;  
    double leftkI = 0;
    double leftkD = .15;
    leftClimb.config_kF(0, leftkF, 20);
    leftClimb.config_kP(0, leftkP, 20);
    leftClimb.config_kI(0, leftkI, 20);
    leftClimb.config_kD(0, leftkD, 20);

    double rightkF = 0.0006;
    double rightkP = .1;
    double rightkI = 0;
    double rightkD = 0;
    rightClimb.config_kF(0, rightkF, 20);
    rightClimb.config_kP(0, rightkP, 20);
    rightClimb.config_kI(0, rightkI, 20);
    rightClimb.config_kD(0, rightkD, 20);

    leftClimb.configForwardSoftLimitThreshold(150000, 0);
    leftClimb.configForwardSoftLimitEnable(true, 0);
    rightClimb.configForwardSoftLimitThreshold(150000,0);
    rightClimb.configForwardSoftLimitEnable(true,0);
  }

  public void moveElevatorStaticUp() {
    leftClimb.set(ControlMode.PercentOutput, elevatorSpeed);
    rightClimb.set(ControlMode.PercentOutput, elevatorSpeed);
  }

  public void setClimberToCurrentPosition() {
    double leftClimbPos = leftClimb.getSelectedSensorPosition();
    double rightClimbPos = rightClimb.getSelectedSensorPosition();
    leftClimb.set(ControlMode.Position, leftClimbPos);
    rightClimb.set(ControlMode.Position, rightClimbPos);
  }
  

  public void moveElevatorStaticDown() {
    leftClimb.set(ControlMode.PercentOutput, -elevatorSpeed);
    rightClimb.set(ControlMode.PercentOutput, -elevatorSpeed);
  }

  public void stopElevator() {
    leftClimb.set(ControlMode.PercentOutput, 0);
    rightClimb.set(ControlMode.PercentOutput, 0);
  }

  public void setLeftPositionToClimbHeight() {
      leftClimb.set(ControlMode.Position, climbHeightExtension);
  }

  public void setRightPositionToClimbHeight() {
    rightClimb.set(ControlMode.Position, climbHeightExtension);
  
  }
  public void setElevatorPositionToClimbHeight() {
    leftClimb.set(ControlMode.Position, climbHeightExtension);
    rightClimb.set(ControlMode.Position, climbHeightExtension);
  }

  public void setPositionToClimbHeightRetraction() {
    leftClimb.set(ControlMode.Position, climbHeightRetraction);
  }

  public void resetClimber() {
    leftClimb.setSelectedSensorPosition(0);
    rightClimb.setSelectedSensorPosition(0);
  }

  public boolean climberAtPosition(double targetPosition) {
    double tolerance = 500;
    double desiredPosition = targetPosition; //calculateTargetVelocity(shooterSpeed);
    return (leftClimb.getSelectedSensorPosition() >= (desiredPosition - tolerance));
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("LeftClimbPosition", leftClimb.getSelectedSensorPosition());
    SmartDashboard.putNumber("RightClimbPosition", rightClimb.getSelectedSensorPosition());
  }
}
  

