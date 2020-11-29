package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants
    {
        // Motor Ports
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        // Encoder Ports and settings
        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final int kEncoderCPR = 360;
        public static final double kWheelDiameterInches = 6;
        public static final double kConversionUnit = 12;
        public static final double kEncoderDistancePerPulse = ((kWheelDiameterInches * Math.PI) / (double) kEncoderCPR) / 12;

        // Gyro Settings
        public static final boolean kGyroReversed = false;

        // PID Settings
        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;

        public static final double kTurnP = 1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        public static final double kMaxTurnRateDegPerS = 100;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;

        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
  }

  public static final class ShooterConstants
  {
        // Motor Ports
        public static final int kIndexerMotorPort = 4;
        public static final int kIntakeMotorPort = 5;
        public static final int kFlywheelMotorPort = 6;

        // Encoder Ports and settings
        public static final int[] kFlywheelEncoderPorts = new int[]{4, 5};
        public static final boolean kFlywheelEncoderReversed = false;

        public static final int kFlywheelEncoderCPR = 4096;

        // Ultrasonic Ports and Settings
        public static final int[] kIntakeUltrasonicPorts = new int[]{6, 7};
        
        // PID Settings
        public static final double kFlywheelP = 1;
        public static final double kFlywheelI = 0;
        public static final double kFlywheelD = 0;
        
        public static final double kFlywheelTargetRPM = 100;

        public static final double kAimXP = 0.1;
        public static final double kAimYP = 0.1;

        public static final double kMinAimGain = 0.05;

        public static final double kAimTolerance = 7;
  }
  
  public static final class ElevatorConstants
  {
        // Motor/Solenoid Ports
        public static final int kElevatorMotor1Port = 7;
        public static final int kElevatorMotor2Port = 8;
        public static final int kElevatorSolenoid1Port = 0;
        public static final int kElevatorSolenoid2Port = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }
}
