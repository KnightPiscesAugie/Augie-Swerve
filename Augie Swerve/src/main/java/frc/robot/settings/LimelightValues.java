package frc.robot.settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.Results;

public class LimelightValues {
    LimelightHelpers.Results llResults;
    public boolean isResultValid;
    int numTags;
    double[] tx = new double[5];
    double[] ty = new double[5];
    double[] ta = new double[5];
    Pose2d botPoseRed;
    Pose2d botPoseBlue;
    private final Translation2d fieldCorner = new Translation2d(16.54, 8.02);

    public LimelightValues(Results llResults, boolean valid){
        this.llResults = llResults;
        this.isResultValid = valid;
        if (isResultValid){
            this.numTags = llResults.targets_Fiducials.length;
            for (int i = 0; i < numTags; i++) {
                this.tx[i] = llResults.targets_Fiducials[i].tx;
                this.ty[i] = llResults.targets_Fiducials[i].ty;
                this.ta[i] = llResults.targets_Fiducials[i].ta;
            }
            this.botPoseRed = llResults.getBotPose2d_wpiRed();
            this.botPoseBlue = llResults.getBotPose2d_wpiBlue();
        }
    }
    public double gettx(int index){return tx[index];}
    public double getty(int index){return ty[index];}
    public double getta(int index){return ta[index];}
    public Pose2d getBotPose(){
        if (DriverStation.getAlliance() == Alliance.Red){
            return botPoseRed;
        } else {
            return botPoseBlue;
        }
    }    
    public boolean isPoseTrustworthy(Pose2d robotPose){
        Pose2d poseEstimate = this.getBotPose();
        if ((poseEstimate.getX()<fieldCorner.getX() && poseEstimate.getY()<fieldCorner.getY())
        && robotPose.getTranslation().getDistance(poseEstimate.getTranslation()) < 0.5)
        return true;
        else return false;
    }
    public double gettimestamp(){
        return (Timer.getFPGATimestamp()
        - (llResults.latency_capture / 1000)
        - (llResults.latency_pipeline / 1000));
    }
}

