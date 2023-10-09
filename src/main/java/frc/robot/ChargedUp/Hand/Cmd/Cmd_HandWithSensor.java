// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Hand.Cmd;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.ChargedUp.ColorSensor.SubSys_ColorSensor;
// //import frc.robot.ChargedUp.DistanceSensor.SubSys_DistanceSensor;
// import frc.robot.ChargedUp.Hand.Const_Hand;
// import frc.robot.ChargedUp.Hand.SubSys_Hand;
// import java.util.function.DoubleSupplier;

// public class Cmd_HandWithSensor extends CommandBase {

//   private final SubSys_Hand handSubSys;
//   private final SubSys_ColorSensor colorSensorSubSys;
//  // private final SubSys_DistanceSensor distanceSensorSubSys;

//   private final DoubleSupplier activationButton;
//   // Public variables
//   private Boolean handisOpen;
//   // Redeclare Constants
//   private static final double kdistanceToCone = Const_Hand.kdistanceToCone;
//   private static final double kdistanceToCube = Const_Hand.kdistanceToCube;
//   /**
//    * Constructor
//    *
//    * @param HandSubsystem
//    * @param ColorSensorSubsystem
//    * @param DistanceSensorSubsystem
//    * @param ActavationButton(boolean)
//    */
//   public Cmd_HandWithSensor(
//       SubSys_Hand handSubSys,
//       SubSys_ColorSensor colorSensorSubSys,
//   //    SubSys_DistanceSensor distanceSensorSubSys,
//       DoubleSupplier activationButton) {

//     this.handSubSys = handSubSys;
//     this.colorSensorSubSys = colorSensorSubSys;
//   //  this.distanceSensorSubSys = distanceSensorSubSys;

//     this.activationButton = activationButton;

//     addRequirements(handSubSys, colorSensorSubSys);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     handisOpen = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     String seenGameElement = colorSensorSubSys.GetTypeOfGameElement();

//  //   double distance = distanceSensorSubSys.GetDistance();

//     if (handisOpen) {
//       if (seenGameElement == "CONE" && distance == kdistanceToCone) {
//         // *Enable LED
//         SmartDashboard.putString("Hand Ready", "Hand is ready with a cone");
//         if (activationButton.getAsDouble() == 1.0) {
//           handSubSys.CloseHand();
//           handisOpen = false;
//         }
//       }

//       if (seenGameElement == "CUBE" && distance == kdistanceToCube) {
//         // *Enable LED
//         SmartDashboard.putString("Hand Ready", "Hand is ready with a cube");
//         if (activationButton.getAsDouble() == 1.0) {
//           handSubSys.CloseHand();
//           handisOpen = false;
//         }
//       } else {
//         SmartDashboard.putString("Hand Ready", "Hand is not ready");
//       }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
