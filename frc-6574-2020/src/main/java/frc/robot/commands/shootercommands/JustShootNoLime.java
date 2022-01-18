/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class JustShootNoLime extends CommandBase {
  /**
   * Creates a new Shoot Command.
   */
  boolean interrupted = false;


  public JustShootNoLime() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //  RobotContainer.aimTurret.schedule();
  //  shooter.raiseHoodForShooting();
  //  Timer.delay(.4);
  //  shooter.extendHoodForLongDistance();
  //  distanceToTarget = 138;
    System.out.println("JustShootNoLime is being initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("JustShootNoLime is running");
    //distanceToTarget = RobotContainer.turret.limelight.getDistanceToTarget(138); // this is in inches
    //RobotContainer.hopper.turnOnForShooting();
    //RobotContainer.shooter.setVelocity(Shooter.shooterSpeed);

    if (RobotContainer.shooter.shooterReady(Shooter.shooterSpeed)) {
        System.out.println("JustShootNoLime condition has been met");
        Timer.delay(.25);
        RobotContainer.shooter.feedAndFire();
<<<<<<< Updated upstream
        Timer.delay(1);
=======
        Timer.delay(3);//1
>>>>>>> Stashed changes
        interrupted = true;
        System.out.println("JustShootNoLime condition has finished"); 
      } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return interrupted && !RobotContainer.oi.operator_rightTrigger.get();
  }
}
