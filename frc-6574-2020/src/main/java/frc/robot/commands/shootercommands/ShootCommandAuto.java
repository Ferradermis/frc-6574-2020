/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootCommandAuto extends SequentialCommandGroup {
  /**
   * Creates a new ShootCommand.
   */
  public ShootCommandAuto() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new PrepForShootingAuto(),

      new JustShootAuto()
    );    
  }
}
