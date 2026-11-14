package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class SwerveModule {

    //TODO: measure drive motor gear ratio
    private static final double DRIVE_GEAR_RATIO = 6.75;

    //TODO: measure wheel diameter (in meters)
    private static final double WHEEL_DIAMETER_METERS = 0.1016;

    //TODO: PID tuning
    private static final double DRIVE_KP = 0.0;
    private static final double STEER_KP = 0.0;
    
    //TODO: voltage cap for PID tuning;
    private static final double MAX_DRIVE_VOLTS = 3.0;
    private static final double MAX_STEER_VOLTS = 2.0;

    private final PIDController drivePID = new PIDController(DRIVE_KP, 0.0, 0.0);
    private final PIDController steerPID = new PIDController(STEER_KP, 0.0, 0.0);
    

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    public SwerveModule(int driveMotorId, int steerMotorId, int encoderId){
        driveMotor = new TalonFX(driveMotorId, "rio");
        steerMotor = new TalonFX(steerMotorId, "rio");
        steerEncoder = new CANcoder(encoderId, "rio");

        steerPID.enableContinuousInput(-Math.PI, Math.PI);

    }

    public Rotation2d getAngle(){

        //get the absolute position of encoder as double
        double rotations = steerEncoder.getAbsolutePosition().getValueAsDouble();

        //convert it to WPILib Rotation2d
        return Rotation2d.fromRotations(rotations);
    }

    private void setDriveSpeed(double targetMetersPerSecond, StatusSignal<AngularVelocity> rotorSignal){

        //calculate current wheel speed in mps
        double currentSpeed = (rotorSignal.getValueAsDouble() / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER_METERS;

        double driveVolts = drivePID.calculate(currentSpeed, targetMetersPerSecond);

        driveVolts = MathUtil.clamp(driveVolts, -MAX_DRIVE_VOLTS, MAX_DRIVE_VOLTS);

        driveMotor.setVoltage(driveVolts);

    }

    private void setDesiredAngle(Rotation2d targetAngle, StatusSignal<Angle> angleSignal){

        //get current angle from encoder
        Rotation2d currentAngle = Rotation2d.fromRotations(angleSignal.getValueAsDouble());

        //calculate PID output
        double steerVolts = steerPID.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        steerVolts = MathUtil.clamp(steerVolts, -MAX_STEER_VOLTS, MAX_STEER_VOLTS);

        steerMotor.setVoltage(steerVolts);
    }

    public void setDesiredState(SwerveModuleState desiredState){
        var rotorSignal = driveMotor.getRotorVelocity();
        var angleSignal = steerEncoder.getAbsolutePosition();

        //check status before applying voltage
        if(!rotorSignal.getStatus().isOK() || !angleSignal.getStatus().isOK()){
            stop();
            return;
        }

        //optimize target
        SwerveModuleState target = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        Rotation2d currentAngle = Rotation2d.fromRotations(angleSignal.getValueAsDouble());
        target.optimize(currentAngle);
        target.cosineScale(currentAngle);


        setDriveSpeed(target.speedMetersPerSecond, rotorSignal);
        setDesiredAngle(target.angle, angleSignal);
    }

    public void stop(){
        driveMotor.stopMotor();
        steerMotor.stopMotor();
        drivePID.reset();
        steerPID.reset();
    }
}
