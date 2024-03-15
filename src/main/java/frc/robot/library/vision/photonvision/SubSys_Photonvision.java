// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.ca

package frc.robot.library.vision.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

import static frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.CAMERA_OFFSET;

public class SubSys_Photonvision implements Subsystem {
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  PhotonPoseEstimator photonPoseEstimator;

  public SubSys_Photonvision(String cameraName) {
    PhotonCamera camera = new PhotonCamera(cameraName);
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, CAMERA_OFFSET);
    photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE);
  }

  @Override
  public void periodic() {
    if (getEstimatedVisionPose2d().isPresent()) {
      SmartDashboard.putString("Vision/Vision Pose Estimate", String.valueOf(getEstimatedVisionPose2d().get().getFirst()));
    } else {
      SmartDashboard.putString("Vision/Vision Pose Estimate", "NONE");
    }
  }

  /**
   * @return A {@link Pair} containing the {@link Pose3d} and the timestamp of the estimation if the MultiTag pose is present from the coprocessor
   * @param previousPose The previous pose of the robot
   */
  public Optional<Pair<Pose3d, Double>> getEstimatedVisionPose3d(Pose2d previousPose) {
    photonPoseEstimator.setLastPose(previousPose);
    Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update();
    return estimate.map(estimatedRobotPose -> new Pair<>(estimatedRobotPose.estimatedPose, estimatedRobotPose.timestampSeconds));
  }

  /**
   * @return A {@link Pair} containing the {@link Pose3d} and the timestamp of the estimation if the MultiTag pose is present from the coprocessor
   */
  public Optional<Pair<Pose3d, Double>> getEstimatedVisionPose3d() {
    Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update();
    return estimate.map(estimatedRobotPose -> new Pair<>(estimatedRobotPose.estimatedPose, estimatedRobotPose.timestampSeconds));
  }

  /**
   * @param previousPose The previous pose of the robot
   * @return A {@link Pair} containing the {@link Pose2d} and the timestamp of the estimation if the MultiTag pose is present from the coprocessor
   */
  public Optional<Pair<Pose2d, Double>> getEstimatedVisionPose2d(Pose2d previousPose) {
    photonPoseEstimator.setLastPose(previousPose);
    Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update();
    return estimate.map(estimatedRobotPose -> new Pair<>(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds));
  }

  /**
   * @return A {@link Pair} containing the {@link Pose2d} and the timestamp of the estimation if the MultiTag pose is present from the coprocessor
   */
  public Optional<Pair<Pose2d, Double>> getEstimatedVisionPose2d() {
    Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update();
    return estimate.map(estimatedRobotPose -> new Pair<>(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds));
  }

}
