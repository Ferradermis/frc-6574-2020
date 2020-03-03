/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbUpandDown extends CommandBase {

  private  Climber climber;
  final double THROTTLE = 1; // controls speed via joystick; useful for test driving
                              // set to 1 for normal drive speed

  public ClimbUpandDown(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  // NOTE  ALL Limelight code and buttonpressed tests should move to own commands

  @Override
  public void execute() {
  

    // Read data from joystick and drive per joystick positioning
    double y = RobotContainer.oi.getOperatorLeftY(); //Operator Controller is used

    // throttle is constant that controls "speed" of robot; helpful in testing in small areas
    climber.move(y);   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}