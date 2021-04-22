/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer; //Might remove, doesn't cause errors yet
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  final double ClimberSpeed = 0.50;

  //public CANSparkMax elevator = new CANSparkMax(RobotMap.ELEVATOR_CAN_ID, MotorType.kBrushless);
  //public CANSparkMax winch = new CANSparkMax(RobotMap.WINCH_CAN_ID, MotorType.kBrushless);
  public DoubleSolenoid climberDeploy = new DoubleSolenoid(RobotMap.CLIMBER_EXTENDER_ID2, RobotMap.CLIMBER_EXTENDER_ID1);

  final DoubleSolenoid.Value DEPLOYED = DoubleSolenoid.Value.kForward;
  final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kReverse;
  
  final double elevatorSpeed = .15;

  public Climber() {
    double rampRate = 0.2;
    int currentLimit = 40; 
   
    //elevator.setOpenLoopRampRate(rampRate); //makes sure it doesn't go too fast when it is about to end?
    //winch.setOpenLoopRampRate(rampRate);

    //elevator.setSmartCurrentLimit(currentLimit); //will stop power if stuck
    //winch.setSmartCurrentLimit(currentLimit);

    //elevator.setInverted(true);
    //elevator.setIdleMode(IdleMode.kBrake);
    //winch.setIdleMode(IdleMode.kBrake);
  }
  public void moveElevatorStaticUp() {
    //elevator.set(elevatorSpeed);
  }

  public void moveElevatorStaticDown() {
    //elevator.set(-elevatorSpeed);
  }

  public void stopElevator() {
    //elevator.set(0);
  }
 /* public void moveElevator(double yLeft) { 
    if ((Math.abs(yLeft) <= 0.1)) {
       elevator.set(0);
       //winch.set(0);
      return;
    }

    elevator.set(yLeft); 
    //winch.set(y);
    //SmartDashboard.putNumber("Climber speed", y);
  }
  */

  public void moveWinch(double yLeft) { 
    if ((Math.abs(yLeft) <= 0.1)) {
       //elevator.set(0);
       //winch.set(0);
      return;
    }

    
    //winch.set(yLeft);
    //SmartDashboard.putNumber("Climber speed", y);
  }



 /*  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  */
}
