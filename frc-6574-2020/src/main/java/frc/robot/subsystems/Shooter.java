/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // shooter/shooter is two falcons -- built in encoder talonFX
  public WPI_TalonFX shooterLeft = new WPI_TalonFX(RobotMap.SHOOTERLEFT_CAN_ID);
  private WPI_TalonFX shooterRight = new WPI_TalonFX(RobotMap.SHOOTERRIGHT_CAN_ID);
  public CANSparkMax feeder = new CANSparkMax(RobotMap.FEEDER_CAN_ID, MotorType.kBrushless);
 
  // hood controller for raising and lowering
  public DoubleSolenoid hoodTrench = new DoubleSolenoid(RobotMap.HOOD_TRENCH_ID1, RobotMap.HOOD_TRENCH_ID2);
  public DoubleSolenoid hoodAngle = new DoubleSolenoid(RobotMap.HOOD_ANGLE_ID2, RobotMap.HOOD_ANGLE_ID1);

  public double enteredShooterVelocity;
  public static double shooterSpeed = 20000; //15500

  public Shooter() {
    configureMotors();
    //raiseHoodForShooting();
    //Timer.delay(.25);
    //extendHoodForLongDistance();
    //SmartDashboard.putNumber("Entered Shooter Velocity", enteredShooterVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    SmartDashboard.putNumber("Current Shooter Velocity", shooterLeft.getSelectedSensorVelocity());
    SmartDashboard.putBoolean("Shooter up to speed?", RobotContainer.shooter.shooterReady(Shooter.shooterSpeed));
  }

  public void spinSmartDash() {
    double maxSpeedThreshold = 13000;
    double enteredShooterVelocity = SmartDashboard.getNumber("Entered Shooter Velocity", 0);
    if (enteredShooterVelocity < 0)
      enteredShooterVelocity = 0;
    if (enteredShooterVelocity > maxSpeedThreshold)
      enteredShooterVelocity = maxSpeedThreshold;
    shooterLeft.set(ControlMode.Velocity, enteredShooterVelocity);

  }
  //public double calculateTargetVelocity(double distance) {
  //   return shooterSpeed; 
  //}

  public boolean shooterReady(double targetShooterSpeed) {
    double tolerance = 500;
    //double targetVelocity_UnitsPer100ms = targetShooterSpeed; //calculateTargetVelocity(shooterSpeed);
    return (Math.abs(shooterLeft.getSelectedSensorVelocity() - targetShooterSpeed) < tolerance);
    //return (shooterLeft.getSelectedSensorVelocity() >= (targetVelocity_UnitsPer100ms - tolerance));
    //return (shooterLeft.getSelectedSensorVelocity() >= (SmartDashboard.getNumber("Entered Shooter Velocity", 0) - tolerance));
  }

  public void feedAndFire() {
    feeder.set(1);
    Timer.delay(0.25);
    RobotContainer.hopper.turnOnForShooting();
  }

  public void setVelocity(double velocity) {
    shooterLeft.set(ControlMode.Velocity, velocity);
  }

  public void stopFeeder() {
    feeder.set(0);
  }

  public void stopFiring() {
    stopFeeder();
    RobotContainer.hopper.turnOff();
  }

  public void stopShooter() {
    stopFiring();
    //shooterLeft.set(ControlMode.PercentOutput, 0); 
    //defaultShooterOn();
    shooterLeft.set(ControlMode.Velocity, shooterSpeed);
  }
 
  public void raiseHoodForShooting() {
      hoodTrench.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerHoodForTrench() {
    hoodAngle.set(DoubleSolenoid.Value.kReverse);
    Timer.delay(.25);
    hoodTrench.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendHoodForLongDistance() {
    // only extend distance hood if trenchHood raised and hoodAngle is down
    if ((hoodTrench.get() == DoubleSolenoid.Value.kForward)){
      hoodAngle.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void retractHoodforShortDistance() {
    // only retractHood if already extended
    if (hoodAngle.get() == DoubleSolenoid.Value.kForward) {
      hoodAngle.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void defaultShooterOn() {
    //shooterLeft.set(ControlMode.PercentOutput, 0.2);
    shooterLeft.set(ControlMode.PercentOutput, .5375); //.5875
    //shooterLeft.set(ControlMode.PercentOutput, 0);
  }

  public void defaultShooterOff() {
    shooterLeft.set(ControlMode.PercentOutput, 0);
  }

  private void configureMotors() {
    // Set up motors
    double rampRate = 0.05; //making this super big temporarily to not make motors unhappy
    //int currentLimit = 35; 
    int feederCurrentLimit = 60;
    //int shooterCurrentLimit = 55; 

    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();

    shooterRight.setInverted(true);
    shooterRight.follow(shooterLeft);

    shooterLeft.configOpenloopRamp(rampRate);
    shooterRight.configOpenloopRamp(rampRate);
    shooterLeft.setNeutralMode(NeutralMode.Coast); // MAKE SURE WE ARE IN COAST MODE
    shooterRight.setNeutralMode(NeutralMode.Coast); // MAKE SURE WE ARE IN COAST MODE


    //CURRENT LIMITS BELOW; SECOND TWO VALUES ON FOLLOWING FOUR LINES ARE ARBITRARY AT THIS STAGE
    
    /*
    shooterLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 55, 55, 0.5));
    shooterRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 55, 55, 0.5));
    shooterLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration (true, 55, 55, 0.5));
    shooterRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration (true, 55, 55, 0.5));
    */

    // SEE CODE FROM: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java

    /* Config sensor used for Primary PID [Velocity] */
    shooterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);




// Config the Velocity closed loop gains in slot0
// PEAK SENSOR VELOCITY on 2020 ROBOT is:
// (kMaxRPM  / 600) * (kSensorUnitsPerRotation)
// PEAK RPM of Motor = 6380 RPM
// PEAK SENSOR VELOCITY = 6380 / 600 * 2048  = 21,777
// CURRENT Gear Ratio = .809:1
// PEAK RPM of Wheel = 7881 RPM



    double kF = .055; 
    double kP = .725;  
    double kI = 0;
    double kD = 0;
    shooterLeft.config_kF(0, kF, 20);
    shooterLeft.config_kP(0, kP, 20);
    shooterLeft.config_kI(0, kI, 20);
    shooterLeft.config_kD(0, kD, 20);

    //feeder.setOpenLoopRampRate(rampRate);
    //feeder.setSmartCurrentLimit(feederCurrentLimit);


  }
}