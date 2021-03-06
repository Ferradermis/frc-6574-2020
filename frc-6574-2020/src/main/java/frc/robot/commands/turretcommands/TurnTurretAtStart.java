/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turretcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnTurretAtStart extends CommandBase {
  /**
   * Creates a new TurnTurretAtStart command.
   */

  double startTime;

  public TurnTurretAtStart() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();   
    RobotContainer.turret.turn(-.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.stopTurning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((Timer.getFPGATimestamp()-startTime) > .3) || (RobotContainer.turret.limelight.hasTarget()));
  }
}
