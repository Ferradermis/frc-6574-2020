// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RapidReact;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoB extends CommandBase {
  /** Creates a new AutoB. */
  public AutoB() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //start in the lower blue section
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //turn right
    //backward to ball
    //turn on intake
    //pick up ball
    //turn on shoot wheel
    //get ball into shooting section 
    //line up whith goal
    //shoot
    //go left
    //turn on intake
    //pick up ball 
    //turn off intake
    // turn on shooting wheel
    // line up whith goal
    // get ball  into wheel
    //shoot 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
