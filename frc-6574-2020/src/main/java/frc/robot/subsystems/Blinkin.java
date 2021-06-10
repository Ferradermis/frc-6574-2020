 
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {

  /* Rev Robotics Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor. 
   *  -1  corresponds to 1000us
   *  0   corresponds to 1500us
   *  +1  corresponds to 2000us
   */
  private static Spark m_blinkin = null;

  /**
   * Creates a new Blinkin LED controller.
   * 
   * @param pwmPort  The PWM port the Blinkin is connected to.
   */
  public Blinkin(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    //larsonScanner();
  }

  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */ 
  public static void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }
  public static void blinkTest() {
    off();
    Timer.delay(10);
    fireMedium();
  }
  
  public static void off() {
    set(.99);
  }

  public static void hotPink() {
    set(.57);
  }

  public static void darkRed() {
    set(.59);
  }

  public static void red() {
    set(.61);
  }

  public static void redOrange(){
  }

  public static void Orange() {
    set(.65);
  }

  public static void Gold() {
    set(.67);
  }

  public static void yellow() {
    set(.69);
  }

  public static void lawnGreen() {
    set(.71);
  }

  public static void lime() {
    set(.73);
  }

  public static void darkGreen() {
    set(.75);
  }

  public static void green() {
    set(.77);
  }

  public static void blueGreen() {
    set(.79);
  }

  public static void aqua() {
    set(.81);
  }

  public static void skyBlue() {
    set(.83);
  }

  public static void darkBlue() {
    set(.85);
  }

  public static void blue() {
    set(.87);
  }

  public static void blueViolet(){
    set(.89);
  }

  public static void violet(){
    set(.91);
  }

  public static void white() {
    set(.93);
  }

  public static void gray() {
    set(.95);
  }

  public static void darkGray() {
    set(.97);
  }

  public static void black() {
    set(.99);
  }

  public static void fireMedium() {
    set(-.59);
  }

  public static void larsonScanner() {
    set(-.35);
  }

  public static void lightChaseRed () {
    set(-.31);
  }

  public static void sineLon () {
    set(-.73);
  }

  public static void rainbow() {
    set(-0.99);
  }

  public static void solid_orange() {
    set(0.65);
  }

  public static void allianceColor() {
    boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    if (isRed == true){
      Blinkin.set(-0.01);
      System.out.println("led RED");
    } else {
      Blinkin.set(0.19);
      System.out.println("led BLUE");
    }
  }
}

