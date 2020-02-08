/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // CREATE and connect to motors
  // shooter is two falcons -- built in encoder talonFX
  private WPI_TalonFX spinner1 = new WPI_TalonFX(RobotMap.SPINNER1_CAN_ID);
  private WPI_TalonFX spinner2 = new WPI_TalonFX(RobotMap.SPINNER2_CAN_ID);

  // loader neo550's
  private CANSparkMax loader = new CANSparkMax(RobotMap.LOADER_CAN_ID, MotorType.kBrushless);
 
  // rotator vexPro775
  // 
  private VictorSPX turretRotator = new VictorSPX(RobotMap.TURRET_CAN_ID);
 
  private double MAXROTATION = 45;

  public Shooter() {
    // Set up motors
    double rampRate = 0.2; //time in seconds to go from 0 to full throttle; 0.2 is selected on feel by drivers for 2019
 
    spinner2.follow(spinner1);
    spinner1.configFactoryDefault();
    spinner2.configFactoryDefault();
    spinner1.configOpenloopRamp(rampRate);
    spinner2.configOpenloopRamp(rampRate);

    int currentLimit = 30; 
 
    loader.setOpenLoopRampRate(rampRate);
    loader.setSmartCurrentLimit(currentLimit);
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 /*   if (shooting == true){
      spinUpTheWheels(distanceFromTarget);
      aim();
      if (aimed==true)
      {
        if (spinningFastEnough==true)
        {
          load();
        }
      }
    
    }
    */
  }

  public void shoot()
  {
  //Timer.delay(0.5);
  }
}