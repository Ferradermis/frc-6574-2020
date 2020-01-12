/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick driver = new Joystick(0);
  public Joystick operator = new Joystick(1);

  // Logitech Variables
  public Button l_xButton = new JoystickButton(driver, 3);
  public Button l_aButton = new JoystickButton(driver, 1);
  public Button l_bButton = new JoystickButton(driver, 2);
  public Button l_yButton = new JoystickButton(driver, 4);
  public Button l_leftBumper = new JoystickButton(driver, 5);
  public Button l_rightBumper = new JoystickButton(driver, 6);
  //public Button l_leftTrigger = new JoystickButton(driver, 7);
  public Button l_backButton = new JoystickButton(driver, 7);
  //public Button l_startButton = new JoystickButton(driver, 8);
  //public Button l_rightTrigger = new JoystickButton(driver, 8);
  //public Button l_backButton = new JoystickButton(driver, 9);
  //public Button l_startButton = new JoystickButton(driver, 10);
  public POVButton l_upDpad = new POVButton(driver, 0);
  public POVButton l_rightDpad = new POVButton(driver, 90);
  public POVButton l_downDpad = new POVButton(driver, 180);
  public POVButton l_leftDpad = new POVButton(driver, 270);

  public double getLogitechLeftX() {
    return -driver.getRawAxis(0);
  }

  public double getLogitechLeftY() {
    return -driver.getRawAxis(1);
  }

  public double getLogitechRightX() {
    return -driver.getRawAxis(2);
  }

  public double getLogitechRightY() {
    return -driver.getRawAxis(5);
  }

  public double getLogitechLeftTrigger() {
    return driver.getRawAxis(2);
  }

  public double getLogitechRightTrigger() {
    return driver.getRawAxis(3);
  }

  public boolean getLogitechLeftBumper() {
    return operator.getRawButton(5);
  }

  public boolean getLogitechRightBumper() {
    return operator.getRawButton(6);
  }

  // operator Variables
  public Button x_xButton = new JoystickButton(operator, 3);
  public Button x_aButton = new JoystickButton(operator, 1);
  public Button x_bButton = new JoystickButton(operator, 2);
  public Button x_yButton = new JoystickButton(operator, 4);
  public Button x_leftBumper = new JoystickButton(operator, 5);
  public Button x_rightBumper = new JoystickButton(operator, 6);
  public Button x_backButton = new JoystickButton(operator, 7);
  public Button x_startButton = new JoystickButton(operator, 8);
  //public Button x_leftTrigger = new JoystickButton(operator, 7);
  //public Button x_rightTrigger = new JoystickButton(operator, 8);
  //public Button x_backButton = new JoystickButton(operator, 9);
  //public Button x_startButton = new JoystickButton(operator, 10);
  public POVButton x_upDpad = new POVButton(operator, 0);
  public POVButton x_rightDpad = new POVButton(operator, 90);
  public POVButton x_downDpad = new POVButton(operator, 180);
  public POVButton x_leftDpad = new POVButton(operator, 270);

  public double getOperatorLeftX() {
    return -operator.getRawAxis(0);
  }

  public double getOperatorLeftY() {
    return -operator.getRawAxis(1);
  }

  public double getOperatorRightX() {
    return -operator.getRawAxis(2);
  }

  public double getOperatorRightY() {
    return -operator.getRawAxis(5);
  }

  public double getOperatorLeftTrigger() {
    return operator.getRawAxis(2);
  }

  public double getOperatorRightTrigger() {
    return operator.getRawAxis(3);
  }

  public boolean getOperatorLeftBumper() {
    return operator.getRawButton(5);
  }

  public boolean getOperatorRightBumper() {
    return operator.getRawButton(6);
  }

  public OI() {
  
    //leftStickButton3.whenPressed(new Shift());
  }

}