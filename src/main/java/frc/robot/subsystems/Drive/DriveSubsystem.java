package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private static final double WHEELBASE_METERS = 0.0;
    private static final double TRACKWIDTH_METERS = 0.0;

    public static double MAX_SPEED_MPS = 1.0;
    public static double MAX_TURN_RAD_PER_SEC = 1.5;

    //module positions relative to the center
    private final SwerveDriveKinematics kinematics =
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

  private final String[] moduleNames = {
    "FrontLeft", "FrontRight", "BackLeft", "BackRight"
  };

  public DriveSubsystem(){};

  public void drive(double forwardMps, double leftMps, double turnRadPerSec){

    //chassis speeds stores all three values: fwd, left and turning(in radians)
    ChassisSpeeds robotSpeeds = new ChassisSpeeds(forwardMps, leftMps, turnRadPerSec);

    //Calculates target speed for each module with applied speed
    SwerveModuleState [] targets = kinematics.toSwerveModuleStates(robotSpeeds);

    //send calculated target speed of each module to dashboard
    for (int i = 0; i < targets.length; i ++){
      String key = "Drive/" + moduleNames[i];

      SmartDashboard.putNumber(key, targets[i].speedMetersPerSecond);
      SmartDashboard.putNumber(key, targets[i].angle.getDegrees());

    }

  }

  public void stop(){
    drive(0.0, 0.0, 0.0);
  }
}
