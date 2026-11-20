package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{

  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.FL_DRIVE_ID, DriveConstants.FL_STEER_ID, DriveConstants.FL_ENCODER_ID);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.FR_DRIVE_ID, DriveConstants.FR_STEER_ID, DriveConstants.FR_ENCODER_ID);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.BL_DRIVE_ID, DriveConstants.BL_STEER_ID, DriveConstants.BL_ENCODER_ID);
  private final SwerveModule backRight = new SwerveModule(DriveConstants.BR_DRIVE_ID, DriveConstants.BR_STEER_ID, DriveConstants.BR_ENCODER_ID);
  private final SwerveModule [] modules = {frontLeft, frontRight, backLeft, backRight};
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.PIGEON2_ID, "rio");
  
  //TODO: delete these after finish tuning PID
  private final NetworkTableEntry driveP = NetworkTableInstance.getDefault().getTable("Tuning").getEntry("driveP");
  private final NetworkTableEntry driveI = NetworkTableInstance.getDefault().getTable("Tuning").getEntry("driveI");
  private final NetworkTableEntry driveD = NetworkTableInstance.getDefault().getTable("Tuning").getEntry("driveD");
  private final NetworkTableEntry steerP = NetworkTableInstance.getDefault().getTable("Tuning").getEntry("steerP");
  private final NetworkTableEntry steerI = NetworkTableInstance.getDefault().getTable("Tuning").getEntry("steerI");
  private final NetworkTableEntry steerD = NetworkTableInstance.getDefault().getTable("Tuning").getEntry("steerD");

  private final NetworkTableEntry maxMetersPerSec = NetworkTableInstance.getDefault().getTable("Speeds").getEntry("maxMetersPerSec");
  private final NetworkTableEntry maxRadPerSec = NetworkTableInstance.getDefault().getTable("Speeds").getEntry("maxRadPerSec");

  public DriveSubsystem(){};

  @Override
  public void periodic() {
    //update max mps and rps
    DriveConstants.MAX_SPEED_METERS_PER_SECOND = maxMetersPerSec.getDouble(4);
    DriveConstants.MAX_TURN_RAD_PER_SECOND = maxRadPerSec.getDouble(7);

    //update PID periodically for tuning
    for (SwerveModule module : modules){
      module.applyDrivePID(driveP.getDouble(SwerveModule.DRIVE_KP), driveI.getDouble(SwerveModule.DRIVE_KI), driveD.getDouble(SwerveModule.DRIVE_KD));
      module.applySteerPID(steerP.getDouble(SwerveModule.STEER_KP), steerI.getDouble(SwerveModule.STEER_KI), steerD.getDouble(SwerveModule.STEER_KD));
    }

      //get yaw angle from gyro (as a status signal)
      var yawSignal = gyro.getYaw();

      SmartDashboard.putBoolean("Drive/GyroConnected", yawSignal.getStatus().isOK());

      if (yawSignal.getStatus().isOK()){
        SmartDashboard.putNumber("Drive/YawAngleDegs", yawSignal.getValueAsDouble());
      }
  }

  public void drive(double forwardMetersPerSecond, double leftMetersPerSecond, double turnRadPerSecond){

    var yawSignal = gyro.getYaw();
    Rotation2d heading = Rotation2d.fromDegrees(yawSignal.getValueAsDouble());

    //chassis speeds stores four values: fwd, left, turning and heading
    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardMetersPerSecond, leftMetersPerSecond, turnRadPerSecond, heading);

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

  public void resetGyro(){
    gyro.setYaw(0.0);
  }

  public void stop(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
}
