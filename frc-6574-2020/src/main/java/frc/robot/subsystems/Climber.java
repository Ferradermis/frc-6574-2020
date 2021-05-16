/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer; //Might remove, doesn't cause errors yet
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  final double ClimberSpeed = 0.50;

  public WPI_TalonFX leftClimb = new WPI_TalonFX(RobotMap.LEFT_CLIMB_CAN_ID);
  public WPI_TalonFX rightClimb = new WPI_TalonFX(RobotMap.LEFT_CLIMB_CAN_ID);

  //public DoubleSolenoid climberDeploy = new DoubleSolenoid(RobotMap.CLIMBER_EXTENDER_ID2, RobotMap.CLIMBER_EXTENDER_ID1);

  final DoubleSolenoid.Value DEPLOYED = DoubleSolenoid.Value.kForward;
  final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kReverse;
  
  final double elevatorSpeed = .25;
  double climbHeightExtension = 54000;
  double climbHeightRetraction = 20000;

  public Climber() {
    double rampRate = 0.2;
    int currentLimit = 30; 
    int currentLimitThreshold = 50;
    double currentLimitThresholdTime = 1.0;

    leftClimb.setSelectedSensorPosition(0);
    
    rightClimb.follow(leftClimb);

    leftClimb.setInverted(true);
    rightClimb.setInverted(true);
    
    rightClimb.setNeutralMode(NeutralMode.Brake);
    leftClimb.setNeutralMode(NeutralMode.Brake);

    rightClimb.configOpenloopRamp(rampRate);
    leftClimb.configOpenloopRamp(rampRate);

    leftClimb.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
    rightClimb.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));

    double kF = 0; 
    double kP = .15;  
    double kI = 0;
    double kD = 0;
    leftClimb.config_kF(0, kF, 20);
    leftClimb.config_kP(0, kP, 20);
    leftClimb.config_kI(0, kI, 20);
    leftClimb.config_kD(0, kD, 20);


  }
  public void moveElevatorStaticUp() {
    leftClimb.set(ControlMode.PercentOutput, elevatorSpeed);
  }

  public void moveElevatorStaticDown() {
    leftClimb.set(ControlMode.PercentOutput, -elevatorSpeed);
  }

  public void stopElevator() {
    leftClimb.set(ControlMode.PercentOutput, 0);
  }
  public void setPositionToClimbHeight() {
    leftClimb.set(ControlMode.Position, climbHeightExtension);
  }

  public void setPositionToClimbHeightRetraction() {
    leftClimb.set(ControlMode.Position, climbHeightRetraction);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Climb Position", leftClimb.getSelectedSensorPosition());

  }
}
  
