/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  //-----PWM ID values-----\\
  public static final int BLINKIN_ID = -1;

  //-----CAN ID values-----\\

  /** Drivetrain */
  public static final int FRONT_LEFT_CAN_ID = 1;
  public static final int BACK_LEFT_CAN_ID = 2;
  public static final int FRONT_RIGHT_CAN_ID =3;
  public static final int BACK_RIGHT_CAN_ID = 4;
  
    

  /** Shooter */
  public static final int SHOOTERLEFT_CAN_ID = 5;
  public static final int SHOOTERRIGHT_CAN_ID = 6;

  /** Climber */
  public static final int LEFT_CLIMB_CAN_ID = 13;
  public static final int RIGHT_CLIMB_CAN_ID = 14;

  /** Intake */
  public static final int INTAKE_MOTOR_CAN_ID = 9;

  /** Turret */
  public static final int TURRET_CAN_ID = 11;  

  /** Hopper */
  public static final int HOPPER_CAN_ID = 10;  
  public static final int FEEDER_CAN_ID = 12;

  
 //-----PCM ID values-----\\

  /** Solenoids */
  public static final int HOOD_TRENCH_ID1 = 2; //( changed from 1 on Thursday the 25th)
  public static final int HOOD_TRENCH_ID2 = 3;
  public static final int HOOD_ANGLE_ID1 = 0;
  public static final int HOOD_ANGLE_ID2 = 1; //(changed from 2 on Thursday the 25th)
  public static final int INTAKE_EXTENDER_ID1 = 4;  
  public static final int INTAKE_EXTENDER_ID2 = 5;
  public static final int CLIMBER_EXTENDER_ID1 = 6;  
  public static final int CLIMBER_EXTENDER_ID2 = 7;
}

//BACK_RIGHT_CAN_ID