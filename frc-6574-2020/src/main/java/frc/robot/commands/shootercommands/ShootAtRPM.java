/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShootAtRPM extends CommandBase {
  /**
   * Creates a new ShootingGreenZone.
   */
  Shooter shooter;
  double RPM;

  public ShootAtRPM(Shooter shooter, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  this.shooter =shooter;
  this.RPM = RPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(RPM);
    if (RobotContainer.turret.limelight.aimedAtTarget() && shooter.shooterReady(RPM)) {
      shooter.feedAndFire();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      if (RobotContainer.aimTurret.isScheduled()) {
        RobotContainer.aimTurret.cancel();
      }
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
