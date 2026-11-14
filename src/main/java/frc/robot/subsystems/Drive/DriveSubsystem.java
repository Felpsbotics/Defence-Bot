package frc.robot.subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{

  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.FL_DRIVE_ID, DriveConstants.FL_STEER_ID, DriveConstants.FL_ENCODER_ID);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.FR_DRIVE_ID, DriveConstants.FR_STEER_ID, DriveConstants.FR_ENCODER_ID);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.BL_DRIVE_ID, DriveConstants.BL_STEER_ID, DriveConstants.BL_ENCODER_ID);
  private final SwerveModule backRight = new SwerveModule(DriveConstants.BR_DRIVE_ID, DriveConstants.BR_STEER_ID, DriveConstants.BR_ENCODER_ID);

  public DriveSubsystem(){};

  public void drive(double forwardMetersPerSecond, double leftMetersPerSecond, double turnRadPerSecond){

    //chassis speeds stores all three values: fwd, left and turning(in radians)
    ChassisSpeeds robotSpeeds = new ChassisSpeeds(forwardMetersPerSecond, leftMetersPerSecond, turnRadPerSecond);

    //Calculates target speed for each module with applied speed
    SwerveModuleState [] targets = DriveConstants.kinematics.toSwerveModuleStates(robotSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targets, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    frontLeft.setDesiredState(targets[0]);
    frontRight.setDesiredState(targets[1]);
    backLeft.setDesiredState(targets[2]);
    backRight.setDesiredState(targets[3]);

    //send calculated target speed of each module to dashboard
    for (int i = 0; i < targets.length; i ++){
      String key = "Drive/" + DriveConstants.moduleNames[i];
      SmartDashboard.putNumber(key + "/TargetSpeedMps", targets[i].speedMetersPerSecond);
      SmartDashboard.putNumber(key + "/TargetAngleDegrees", targets[i].angle.getDegrees());
    }
  }

  public void stop(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
}
