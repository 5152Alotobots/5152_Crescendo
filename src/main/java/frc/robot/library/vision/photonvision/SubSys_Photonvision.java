// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.ca

package frc.robot.library.vision.photonvision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

import static frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.cameraOffset;

public class SubSys_Photonvision implements Subsystem {

  private PhotonCamera camera;

  public SubSys_Photonvision(String cameraName) {
    camera = new PhotonCamera(cameraName);
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
   * @return A {@link Pair} containing the {@link Transform3d} and the timestamp of the estimation if the MultiTag pose is present from the coprocessor
   */
  public Optional<Pair<Transform3d, Double>> getEstimatedVisionRobotTransform() {
    // Get result and check if multitag pose is present
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.getMultiTagResult().estimatedPose.isPresent) {
      // Get transform pose to camera
      Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
      // Calculate robot transform
      Transform3d robotTransform = fieldToCamera.plus(cameraOffset.inverse());
      return Optional.of(new Pair<>(robotTransform, result.getTimestampSeconds()));
    }
    // If not present return empty
    return Optional.empty();
  }

  /**
   * @return A {@link Pair} containing the {@link Pose3d} and the timestamp of the estimation if the MultiTag pose is present from the coprocessor
   */
  public Optional<Pair<Pose3d, Double>> getEstimatedVisionPose3d() {
    if (getEstimatedVisionRobotTransform().isPresent()) {
      Transform3d robotTransform = getEstimatedVisionRobotTransform().get().getFirst();
      return Optional.of(new Pair<>(new Pose3d(robotTransform.getX(), robotTransform.getY(), robotTransform.getZ(), robotTransform.getRotation()), getEstimatedVisionRobotTransform().get().getSecond()));
    }
    // If not present return empty
    return Optional.empty();
  }

  /**
   * @return A {@link Pair} containing the {@link Pose2d} and the timestamp of the estimation if the MultiTag pose is present from the coprocessor
   */
  public Optional<Pair<Pose2d, Double>> getEstimatedVisionPose2d() {
    if (getEstimatedVisionRobotTransform().isPresent()) {
      Transform3d robotTransform = getEstimatedVisionRobotTransform().get().getFirst();
      return Optional.of(new Pair<>(new Pose2d(robotTransform.getX(), robotTransform.getY(), robotTransform.getRotation().toRotation2d()), getEstimatedVisionRobotTransform().get().getSecond()));
    }
    // If not present return empty
    return Optional.empty();
  }

}
