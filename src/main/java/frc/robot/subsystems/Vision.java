package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static frc.robot.Constants.VisionConstants.PhotonVision.kLocalizationStrategy;
import static frc.robot.Constants.VisionConstants.PhotonVision.kFallbackStrategy;
import static frc.robot.Constants.VisionConstants.PhotonVision.kSingleTagDefaultStdDevs;
import static frc.robot.Constants.VisionConstants.PhotonVision.kMultiTagDefaultStdDevs;

public class Vision {

    public class CamStruct {
        public PhotonCamera camera;
        public Transform3d offset;
        public PhotonPoseEstimator estimator;
        public CamStruct(String name,Transform3d offset) {
            this.camera = new PhotonCamera(name);
            this.offset = offset;
            this.estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), kLocalizationStrategy, offset);
            this.estimator.setMultiTagFallbackStrategy(kFallbackStrategy);
        }
    }

    //A Consumer that accepts a Pose3d and a Matrix of Standard Deviations, usually should call addVisionMeasurements() on a SwerveDrivePoseEstimator3d
    private final BiConsumer<Pose2d,Matrix<N3,N1>> measurementConsumer;

    HashMap<String,CamStruct> Cameras; //Hashmap of cameras used for Localization
    
    public Vision(BiConsumer<Pose2d,Matrix<N3,N1>> measurementConsumer) {
        this.measurementConsumer = measurementConsumer;
    }

    public void addCamera(String name, Transform3d offset) {
        Cameras.put(name,new CamStruct(name,offset));
    }

    

    private Matrix<N3,N1> generateStdDevs(CamStruct cam, Optional<EstimatedRobotPose> pose, List<PhotonTrackedTarget> targets) {
        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagDefaultStdDevs;
        int numTags = 0;
        double avgDist = 0;
        if (pose.isEmpty()) {
            return kSingleTagDefaultStdDevs;
        }
        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = cam.estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {continue;}
            numTags++;
            avgDist +=
                tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(pose
                        .get()
                        .estimatedPose
                        .toPose2d()
                        .getTranslation()
                    );
        }
        if (numTags != 0) {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) estStdDevs = kMultiTagDefaultStdDevs;
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4)
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return estStdDevs;
    }

    



}

