/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.RobotContainer;

public class StopShooting extends InstantCommand {
  /**
   * Creates a new StopShooting Command.
   */
  Shooter shooter;

  public StopShooting(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shoot.cancel();
    shooter.retractHoodforShortDistance();
    RobotContainer.aimTurret.cancel();
    shooter.stopShooter();
    shooter.stopFeeder();
    RobotContainer.turret.resetTurretForward();
    shooter.lowerHoodForTrench();

  }
}
