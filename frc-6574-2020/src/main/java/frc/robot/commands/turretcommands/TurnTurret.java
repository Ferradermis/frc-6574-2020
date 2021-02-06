/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turretcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurnTurret extends CommandBase {

  private Turret turret;
  final double THROTTLE = .5; // controls speed via joystick; useful for testing

  public TurnTurret(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
     //Read data from joystick and turn turret per joystick positioning
    double x = RobotContainer.oi.getOperatorRightX();
    if ((Math.abs(x) <= 0.25)) {
      x = 0;
    }

    turret.turn(x*THROTTLE);   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    turret.turn(0);
  }

}