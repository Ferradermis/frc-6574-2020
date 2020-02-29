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
import frc.robot.commands.TurnTurret;
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
  public final TurnTurret turnTurret = new TurnTurret(turret);
  public static final AimTurret aimTurret = new AimTurret(turret);
  public static final Shoot shoot = new Shoot(shooter);

  public static SendableChooser<CommandBase> autochooser = new SendableChooser<CommandBase>();

  public RobotContainer() {

    driveTrain.setDefaultCommand(arcadeDrive);
    turret.setDefaultCommand(turnTurret);

    SmartDashboard.putNumber("Delay Start of Auto: ", 0.0);
    autochooser.setDefaultOption("Test Plan", new AutoTest(driveTrain));
    autochooser.addOption("Target S3 + I3 Trench + S3 balls", new AutoPlanAShoots6(driveTrain));
    autochooser.addOption("Opponent trench I2 S5 balls", new AutoPlanBShoots5(driveTrain));
    autochooser.addOption("Move off Initiation line", new AutoPlanCMovesOffLine(driveTrain));
    SmartDashboard.putData("Autonomous Chooser", autochooser);

    SmartDashboard.putNumber("Shooter Speed", 0.5);
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
    * INTENDED MAPPINGS:
    Driver joystick LEFT = drivetrain forward and reverse
    Driver joystick RIGHT  = drivetrain left and right
    Right Trigger  = start automatic shoot sequence
    Driver button B = AVAILABLE
    Driver button A = run feed and fire when pressed, stop when released
    Driver button X = AVAILABLE
    Driver button Y = AVAILABLE

    Driver POV's = AVAILABLE
    
    WE CAN TRY TO RUMBLE ON CERTAIN CONDITIONS:
    (e.g. end game, shooting, no target, etc..)
*/
  oi.driver_bButton.whenPressed(shoot).whenReleased(()->shoot.cancel());
  oi.driver_rightTrigger.whenPressed(()->shooter.feedAndFire())
  .whenReleased(()->shooter.stopFiring());

/*
    Operator joystick LEFT = raise and lower climber
    Operator joystick RIGHT  = turn turret counterclockwise and clockwise
    Operator button X = deploy intake, spin (& spin hopper) and retract intake
    Operator button Y = reverse spin intake on press; forward spin on release
    Operator button B = AVAILABLE
    Operator button A = AVAILABLE
    Operator POV up = raise trench hood (assumes not auto-shooting)
    Operator POV right = extend distance hood (assumes not auto-shooting)
    Operator POV left = retract distance hood (assume not auto-shooting)
    Operator POV down = retract distance hood and lower trench hood (assumes not auto-shooting)
    Operator right bumper = spin hopper forward (for testing or unjamming balls in game play)
    Operator left bumper = spin hopper backward (for testing or unjamming balls in game play)
    Operator right trigger = spin shooter on press; stop shooter on release

    Could use operator POV to control velocity of shooter; turn hopper, etc..
    */

    oi.operator_xButton.whenPressed(()->intake.deployOrRetract());
    oi.operator_yButton.whenPressed(()->intake.reverseOn())
                .whenReleased(()->intake.turnOff()); 
    oi.operator_aButton.whenPressed(()->shooter.testspin())
                .whenReleased(()->shooter.teststop());
   oi.operator_rightTrigger.whenPressed(()->shooter.testspin())
                .whenReleased(()->shooter.teststop());
    oi.operator_rightBumper.whenPressed(()->hopper.turnOnForIntake())
                .whenReleased(()->hopper.turnOff());
    oi.operator_leftBumper.whenPressed(()->hopper.reverseForIntake())
                .whenReleased(()->hopper.turnOff());
    oi.operator_upDpad.whenPressed(()->shooter.raiseHoodForShooting());
    oi.operator_downDpad.whenPressed(()->shooter.lowerHoodForTrench());
    oi.operator_rightDpad.whenPressed(()->shooter.extendHoodForLongDistance());
    oi.operator_upDpad.whenPressed(()->shooter.retractHoodforShortDistance());
    


     
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
