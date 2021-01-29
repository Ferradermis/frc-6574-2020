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

  /*
    PWM port values
  */
  public static final int BLINKIN_ID = -1;

  /*
    CAN ID values
  */
  /** Front left drive train motor CAN ID. */
  public static final int FRONT_LEFT_CAN_ID = 1;
  /** Back left drive train motor CAN ID. */
  public static final int BACK_LEFT_CAN_ID = 2;
  /** Front right drive train motor CAN ID. */
  public static final int FRONT_RIGHT_CAN_ID =3;
  /** Back right drive train motor CAN ID. */
  public static final int BACK_RIGHT_CAN_ID = 4;

  /** Shooter motors CAN ID */
  public static final int SHOOTERLEFT_CAN_ID = 5;
  public static final int SHOOTERRIGHT_CAN_ID = 6;

  /** Intake motor CAN ID. */
  public static final int INTAKE_MOTOR_CAN_ID = 9;
  
  /** Hopper motor CAN ID. */
  public static final int HOPPER_CAN_ID = 10;  
  public static final int AGITATOR_CAN_ID = 14;

  public static final int FEEDER_CAN_ID = 12;
 /**  Turret motor CAN ID. */
 public static final int TURRET_CAN_ID = 11;  

 /** Climb motor CAN ID. */
 public static final int CLIMB1_CAN_ID = 7;  
 public static final int CLIMB2_CAN_ID = 8;

  /*
    PCM ID values
  */

  /** Drive train shifter ID. */
//  public static final int SHIFTER_ID_BACK = 0;
//  public static final int SHIFTER_ID_FORWARD = 1;

  /** Hatch intake solenoid ID. */
  public static final int HOOD_TRENCH_ID1 = 1;
  public static final int HOOD_TRENCH_ID2 = 3;
  public static final int HOOD_ANGLE_ID1 = 0;
  public static final int HOOD_ANGLE_ID2 = 2;
  public static final int INTAKE_EXTENDER_ID1 = 4;  
  public static final int INTAKE_EXTENDER_ID2 = 5;
  public static final int CLIMBER_EXTENDER_ID1 = 6;  
  public static final int CLIMBER_EXTENDER_ID2 = 7;
}