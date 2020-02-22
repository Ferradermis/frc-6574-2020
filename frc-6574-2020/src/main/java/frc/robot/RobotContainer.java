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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoPlanAShoots6;
import frc.robot.commands.AutoPlanBShoots5;
import frc.robot.commands.AutoPlanCMovesOffLine;
import frc.robot.commands.AutoTest;
import frc.robot.commands.AimTurret;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;


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

  public static final Shooter shooter = new Shooter();
  public static final Intake intake = new Intake();
  public static final Hopper hopper = new Hopper();
  public static final Turret turret = new Turret();

  public static final Compressor compressor = new Compressor();
  
 // public static Spark leds = new Spark(0);
  
  //Commands
  public final ArcadeDrive arcadeDrive = new ArcadeDrive(driveTrain);
  public static final AimTurret aimTurret = new AimTurret(turret);
  public static final Shoot shoot = new Shoot(shooter);

  public static SendableChooser<CommandBase> autochooser = new SendableChooser<CommandBase>();

  public RobotContainer() {

    configureButtonBindings();
    driveTrain.setDefaultCommand(arcadeDrive);

    SmartDashboard.putNumber("Delay Start of Auto: ", 0.0);
    autochooser.setDefaultOption("Test Plan", new AutoTest(driveTrain));
    autochooser.addOption("Front of target 3 balls", new AutoPlanAShoots6(driveTrain));
    autochooser.addOption("Front of opponent port 2 balls", new AutoPlanBShoots5(driveTrain));
    autochooser.addOption("Moves off Initiation line", new AutoPlanCMovesOffLine(driveTrain));
    SmartDashboard.putData("Autonomous Chooser", autochooser);

    SmartDashboard.putNumber("Shooter Speed", 0.5);
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
                
    oi.driver_yButton.whenPressed(autochooser.getSelected());  // remove for gameplay

  //  oi.operator_bButton.whenPressed(()->shooter.extendHoodForLongDistance())
  //  .whenReleased(()->shooter.retractHoodforShortDistance());

    oi.driver_aButton.whenPressed(()->shooter.testspin())
    .whenReleased(()->shooter.teststop());

    oi.driver_leftBumper.whenPressed(()->turret.testTurnTurret());

    // Operator Buttons
   // oi.operator_bButton.whenPressed(()->intake.turnOn())
     //           .whenReleased(()->intake.turnOff());

//    oi.operator_aButton.whenPressed(()->intake.deployOrRetract());

//    oi.operator_yButton.whenPressed(()->intake.reverseOn())
//                .whenReleased(()->intake.turnOff()); 
     oi.driver_xButton.whenPressed(()->shooter.feedAndFire())
     .whenReleased(()->shooter.stopFeeder());
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
    return autochooser.getSelected();
  }
}
