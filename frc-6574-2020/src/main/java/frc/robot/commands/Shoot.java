/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.RobotContainer;

public class Shoot extends CommandBase {
  /**
   * Creates a new Shoot Command.
   */
  Shooter shooter;
  double distanceToTarget;

  public Shoot(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.aimTurret.schedule();
    shooter.raiseHoodForShooting();
    distanceToTarget = 144;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotContainer.turret.limelight.hasTarget())) {
      distanceToTarget = RobotContainer.turret.limelight.getDistanceToTarget(); // this is in inches
    }

    shooter.spin(distanceToTarget);
      
    
  //  if (distanceToTarget > shooter.hoodNeededDistance) {
  //      shooter.extendHoodForLongDistance();
  //  } else {
  //      shooter.retractHoodforShortDistance();
  //  }

    if (RobotContainer.turret.limelight.aimedAtTarget() && shooter.shooterReady(distanceToTarget-72)) {
        shooter.feedAndFire();
    } 
    //else { // shooting, but not aimed or not ready
    //    shooter.stopFeeder();
   // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.retractHoodforShortDistance();
    RobotContainer.aimTurret.cancel();
    shooter.stopShooter();
    shooter.stopFeeder();
    RobotContainer.hopper.turnOff();
    RobotContainer.turret.resetTurretForward();
    shooter.lowerHoodForTrench();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
