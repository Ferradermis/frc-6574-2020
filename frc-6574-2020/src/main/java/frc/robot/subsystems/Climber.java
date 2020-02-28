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
//import frc.robot.RobotContainer; //Might remove, doesn't cause errors yet
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  final double ClimberSpeed = 0.50;

  public CANSparkMax Climb1Motor = new CANSparkMax(RobotMap.CLIMB1_CAN_ID, MotorType.kBrushless);
  public CANSparkMax Climb2Motor = new CANSparkMax(RobotMap.CLIMB2_CAN_ID, MotorType.kBrushless);
  

  
  public Climber() {
    double rampRate = 0.2;
    int currentLimit = 30; 
   
    Climb1Motor.setOpenLoopRampRate(rampRate); //makes sure it doesn't go too fast when it is about to end?
    Climb2Motor.setOpenLoopRampRate(rampRate);

    Climb1Motor.setSmartCurrentLimit(currentLimit); //will stop power if stuck
    Climb2Motor.setSmartCurrentLimit(currentLimit);
  }

  public void move(double y) { 
    Climb1Motor.set(ClimberSpeed);
    Climb2Motor.set(ClimberSpeed);
  }

  /*
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  */
}
