/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.RunGyroAutonomousSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  
  //Subsystems
  /** */ 
  public static final DriveTrain driveTrain = new DriveTrain();
  public static final OI oi = new OI(); //Phase out
  public static final Limelight limelight = new Limelight();
  public static final Shooter shooter = new Shooter();
  public static final Intake intake = new Intake();
  public static final Hopper hopper = new Hopper();
  public static final Compressor compressor = new Compressor();
  public static SendableChooser<String> autochooser = new SendableChooser<String>();
  
 // public static Spark leds = new Spark(0);
  
  //Commands
  public final ArcadeDrive arcadeDrive = new ArcadeDrive(driveTrain);

  public RobotContainer() {
    configureButtonBindings();
    driveTrain.setDefaultCommand(arcadeDrive);

    autochooser.setDefaultOption("Test Plan", new String("Test"));
    autochooser.addOption("front of target 3 balls", "Plan A");
    autochooser.addOption("front of opponent port 2 balls", "Plan B");
    SmartDashboard.putData("Autonomous Chooser", autochooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver Buttons
  //  oi.operator_xButton.whenPressed(()->shooter.raiseHoodForShooting())
  //              .whenReleased(()->shooter.lowerHoodForTrench());
                
    oi.driver_yButton.whenPressed(new RunGyroAutonomousSequence(driveTrain));  // remove for gameplay

  //  oi.operator_bButton.whenPressed(()->shooter.extendHoodForLongDistance())
  //  .whenReleased(()->shooter.retractHoodforShortDistance());

 //   oi.driver_aButton.whenPressed(()->shooter.testspin())
 //   .whenReleased(()->shooter.teststop());

    oi.driver_leftBumper.whenPressed(()->shooter.testTurnTurret());

    // Operator Buttons
   // oi.operator_bButton.whenPressed(()->intake.turnOn())
     //           .whenReleased(()->intake.turnOff());

//    oi.operator_aButton.whenPressed(()->intake.deployOrRetract());

//    oi.operator_yButton.whenPressed(()->intake.reverseOn())
//                .whenReleased(()->intake.turnOff()); 
//     oi.operator_xButton.whenPressed(()->shooter.feedAndFire())
//     .whenReleased(()->shooter.stopFeeder());
//     oi.operator_leftBumper.whenPressed(()->hopper.turnOn())
//     .whenReleased(()->hopper.turnOff());
//     oi.operator_rightBumper.whenPressed(()->hopper.reverseOn())
//     .whenReleased(()->hopper.turnOff());

     
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new RunGyroAutonomousSequence(driveTrain);
  }
}
