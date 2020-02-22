/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Shoot extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private boolean shooting = false;
  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((shooting == true)&&(RobotContainer.turret.limelight.hasTarget())) {
      RobotContainer.shooter.raiseHoodForShooting();
      RobotContainer.shooter.spin(RobotContainer.turret.getDistanceToTarget());
      RobotContainer.turret.aim();
    
      if (RobotContainer.turret.getDistanceToTarget()>RobotContainer.shooter.hoodNeededDistance) {
        RobotContainer.shooter.extendHoodForLongDistance();
      } else {
        RobotContainer.shooter.retractHoodforShortDistance();
      }

      if (RobotContainer.turret.aimed() && RobotContainer.shooter.shooterReady(RobotContainer.turret.getDistanceToTarget())) {
        RobotContainer.shooter.feedAndFire();
        } else { // shooting, but not aimed or not ready
          RobotContainer.shooter.stopFeeder();
        }
    } else { // not shooting or no target
      shooting = false; // stops all motors
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.stopAiming();
    RobotContainer.shooter.stopShooter();
    RobotContainer.shooter.stopFeeder();
    RobotContainer.shooter.retractHoodforShortDistance();
    RobotContainer.shooter.lowerHoodForTrench();
    RobotContainer.turret.resetTurretForward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooting;
  }
}
