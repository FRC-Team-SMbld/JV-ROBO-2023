// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants{
    public static final int BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final int BEAM_BALANACED_DRIVE_KP = 0;
    public static final int BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 0;
    public static final int BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 0;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveTrainConstants{  
    public static final int rightMotor1 = 12;
    public static final int rightMotor2 = 23;
    public static final int leftMotor1 = 21;
    public static final int leftMotor2 = 29;
    public static final int maxSpeed = 0;
    public static final int maxAngularVelocity = 0;
  }
  public static class ClawSubsystemConstants{
    public static final int rightPnuematicInput = 6;
    // public static final int leftPnuematicInput = 2;
    public static final int rightPnuematicOutput = 7;
    // public static final int leftPnuematicOutput = 3;
    public static final double minPressure = 50;
    public static final double maxPressure = 50;


  }
  public static class ArmSubsystemConstants{
    public static final int rightMotor = 15;
  }
  public static class TriangleSubsystemConstants{
    public static final int rightMotor = 50; 
    public static final int leftMotor = 14; // FIXME DEEZ NUTS i love u, second motor
  }
}
