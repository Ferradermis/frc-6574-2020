/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climbercommands;

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
    RobotContainer.shooter.defaultShooterOff();
    climber.retract();
  }

  // Called repeatedly when this Command is scheduled to run
  // NOTE  ALL Limelight code and buttonpressed tests should move to own commands

  @Override
  public void execute() {
  

    // Read data from joystick and drive per joystick positioning
    if (climber.retracted()){
      double yLeft = RobotContainer.oi.getOperatorLeftY(); //Operator Controller is used
      //double yRight = RobotContainer.oi.getOperatorRightY();
      //climber.moveElevator(yRight);
      climber.moveWinch(yLeft);
    }   
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }

}