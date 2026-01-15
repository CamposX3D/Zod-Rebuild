package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ShooterPhysics;



public class ShooterSubsystem extends SubsystemBase{

    private final TalonFX shooter = 
        new TalonFX(ShooterConstants.SHOOTER_ID);

    private final VelocityVoltage velocityRequest = 
        new VelocityVoltage(0);

    private double targetRPM = 0.0;


    public ShooterSubsystem(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.Slot0.kV = 0;
        config.Slot0.kS = 0;

        config.Feedback.SensorToMechanismRatio = 1.0;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        shooter.getConfigurator().apply(config);
        shooter.setNeutralMode(NeutralModeValue.Coast);

        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    @Override
    public void periodic(){
        shooter.setControl(
            velocityRequest.withVelocity(rpmToRps(targetRPM))
        );


        double[] distanceZ = LimelightHelpers.getCameraPose_TargetSpace( "limelight");
        SmartDashboard.putNumber("DistanceToTarget(M)", distanceZ[2]);
    }

    public void setRPM(double rpm){
        targetRPM = rpm;
    }

    public boolean atSetPoint(){
        double error = Math.abs(
            shooter.getVelocity().getValueAsDouble()
            -rpmToRps(targetRPM)
        );
        return error < 1.0; //RPS
    }

    private double rpmToRps(double rpm){
        return rpm / 60.0;
    }


    public void setRPMFromDistance(double distance){

        if(distance <= 0){
            return;
        }

        double heightDiff = 
            ShooterConstants.TARGET_HEIGHT - ShooterConstants.SHOOTER_EXIT_HEIGHT;

        double rpm = ShooterPhysics.calculateRPM(
            distance,
            heightDiff,
            ShooterConstants.SHOOTER_ANGLE_RAD,
            ShooterConstants.WHEEL_RADIUS
            );

        rpm = MathUtil.clamp(
            rpm,
            ShooterConstants.MIN_RPM,
            ShooterConstants.MAX_RPM
        );

        setRPM(rpm);
    }
}
