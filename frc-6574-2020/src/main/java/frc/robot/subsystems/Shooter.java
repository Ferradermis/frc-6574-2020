/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

<<<<<<< Updated upstream
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
=======
import edu.wpi.first.wpilibj2.command.SubsystemBase;
>>>>>>> Stashed changes

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  // CREATE and connect to motors

  public Shooter() {
// Set up motors

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot()
  {
<<<<<<< Updated upstream
    RobotContainer.leds.set(.99);
    Timer.delay(5.0);
=======
  //Timer.delay(0.5);
>>>>>>> Stashed changes
  }
}