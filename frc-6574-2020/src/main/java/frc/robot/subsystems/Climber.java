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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer; //Might remove, doesn't cause errors yet
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  final double ClimberSpeed = 0.50;

  public CANSparkMax Climb1Motor = new CANSparkMax(RobotMap.CLIMB1_CAN_ID, MotorType.kBrushless);
  public CANSparkMax Climb2Motor = new CANSparkMax(RobotMap.CLIMB2_CAN_ID, MotorType.kBrushless);
  public DoubleSolenoid climberDeploy = new DoubleSolenoid(RobotMap.CLIMBER_EXTENDER_ID2, RobotMap.CLIMBER_EXTENDER_ID1);

  final DoubleSolenoid.Value DEPLOYED = DoubleSolenoid.Value.kForward;
  final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kReverse;
  
  public Climber() {
    double rampRate = 0.2;
    int currentLimit = 30; 
   
    Climb1Motor.setOpenLoopRampRate(rampRate); //makes sure it doesn't go too fast when it is about to end?
    Climb2Motor.setOpenLoopRampRate(rampRate);

    Climb1Motor.setSmartCurrentLimit(currentLimit); //will stop power if stuck
    Climb2Motor.setSmartCurrentLimit(currentLimit);

    Climb2Motor.setInverted(true);

//    Climb2Motor.follow(Climb1Motor);
  }

  public void move(double y) { 
    if ((Math.abs(y) <= 0.1)) {
       Climb1Motor.set(0);
       Climb2Motor.set(0);
      return;
    }

    Climb1Motor.set(y);
    Climb2Motor.set(y);
    SmartDashboard.putNumber("Climber speed", y);
  }

  public void deploy() {
    climberDeploy.set(DEPLOYED);
  }

  public void retract() {
    climberDeploy.set(RETRACTED);
  }

  public boolean retracted() {
    return (climberDeploy.get() == RETRACTED);
  }

 /*  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  */
}
