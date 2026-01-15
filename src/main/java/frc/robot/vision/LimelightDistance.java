package frc.robot.vision;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;

public class LimelightDistance {

    private static final double CAMERA_HEIGHT = 0.85; //altura limelight
    private static final double TARGET_HEIGHT = 1.8288; //altura do hub
    private static final double CAMERA_ANGLE = Math.toRadians(30); //ângulo da limelight

    private static  LimelightTarget_Fiducial limelightFiducial;


    public static double getDistance(){
        if(!LimelightHelpers.getTV("limelight")) {
            return -1;
        }

        double ty = LimelightHelpers.getTY("limelight");

        return(TARGET_HEIGHT - CAMERA_HEIGHT) / 
            Math.tan(CAMERA_ANGLE + Math.toRadians(ty));
    }

    public double getDistance2(){

       return limelightFiducial.getCameraPose_TargetSpace().getZ();
    }
}
