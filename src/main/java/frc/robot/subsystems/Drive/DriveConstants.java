package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DriveConstants {
    //TODO: assgin IDs
    public static final int FL_DRIVE_ID = 0;
    public static final int FL_STEER_ID = 1;
    public static final int FL_ENCODER_ID = 2;
    public static final int FR_DRIVE_ID = 3;
    public static final int FR_STEER_ID = 4;
    public static final int FR_ENCODER_ID = 5;
    public static final int BL_DRIVE_ID = 6;
    public static final int BL_STEER_ID = 7;
    public static final int BL_ENCODER_ID = 8;
    public static final int BR_DRIVE_ID = 9;
    public static final int BR_STEER_ID = 10;
    public static final int BR_ENCODER_ID = 11;

    public static final double WHEELBASE_METERS = 0.0;
    public static final double TRACKWIDTH_METERS = 0.0;

    public static double MAX_SPEED_METERS_PER_SECOND = 1.0;
    public static double MAX_TURN_RAD_PER_SECOND = 1.5;

    //module positions relative to the center
    public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
            //FL
          new Translation2d(
              WHEELBASE_METERS / 2, TRACKWIDTH_METERS / 2),
            //FR
          new Translation2d(
              WHEELBASE_METERS / 2, -TRACKWIDTH_METERS / 2),
            //BL
          new Translation2d(
              -WHEELBASE_METERS / 2, TRACKWIDTH_METERS / 2),
            //BR
          new Translation2d(
              -WHEELBASE_METERS / 2, -TRACKWIDTH_METERS / 2));

  public static final String[] moduleNames = {
    "FrontLeft", "FrontRight", "BackLeft", "BackRight"
  };
}
