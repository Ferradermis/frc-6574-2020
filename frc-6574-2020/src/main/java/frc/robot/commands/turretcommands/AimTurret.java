/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turretcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class AimTurret extends CommandBase {
  /**
   * Creates a new AimTurret command.
   */

  private double turnKP = .06;
  private double simpleFF =.03;
  private double threshold = .25;
  private double offset = -.35;
//  private double MAXROTATION = 45;

  Turret turret;

  public AimTurret(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    this.turret = turret;
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (turret.limelight.hasTarget()) {
      double angleX = turret.limelight.getAngleX();

      if (angleX > threshold + offset) {
        turret.turn((angleX*turnKP) + simpleFF); // copysign Deleted (what is this), .04 added as simple feedforward

      }
      else if (angleX < threshold + offset) {
        turret.turn((angleX*turnKP) - simpleFF);
      }
    } else {
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
