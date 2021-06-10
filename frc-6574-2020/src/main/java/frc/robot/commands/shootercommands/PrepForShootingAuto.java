/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.turretcommands.AimTurretAuto;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class PrepForShootingAuto extends SequentialCommandGroup {
  /**
   * Creates a new PrepForShooting.
   */
  public PrepForShootingAuto() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new InstantCommand(()->RobotContainer.shooter.setVelocity(Shooter.shooterSpeed)),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(()->RobotContainer.shooter.raiseHoodForShooting()),
          
          new WaitCommand(.1),
          new ExtendHood()
         ),
        new AimTurretAuto()
      )
     );
  }
}
