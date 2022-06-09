// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// from https://github.com/Mechanical-Advantage/RobotCode2022

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.GeomUtil;

/** Contains various field dimensions and useful side points. All dimensions are in meters. */
public final class FieldConstants {

  private FieldConstants() {
    throw new IllegalStateException("constant class");
  }

  // Field dimensions
  public static final double FIELD_LENGTH = Units.inchesToMeters(54.0 * 12.0);
  public static final double FIELD_WIDTH = Units.inchesToMeters(27.0 * 12.0);
  public static final double HANGAR_LENGTH = Units.inchesToMeters(128.75);
  public static final double HANGAR_WIDTH = Units.inchesToMeters(116.0);

  // Vision target
  public static final double VISION_TARGET_DIAMETER = Units.inchesToMeters(4.0 * 12.0 + 5.375);
  public static final double VISION_TARGET_HEIGHT_LOWER =
      Units.inchesToMeters(8.0 * 12.0 + 5.625); // Bottom of tape
  public static final double VISION_TARGET_HEIGHT_UPPER =
      VISION_TARGET_HEIGHT_LOWER + Units.inchesToMeters(2.0); // Top of tape

  // Dimensions of hub and tarmac
  public static final Rotation2d CENTER_LINE_ANGLE = Rotation2d.fromDegrees(66.0);
  public static final Translation2d HUB_CENTER =
      new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);
  public static final double TARMAC_INNER_DIAMETER = Units.inchesToMeters(219.25);
  public static final double TARMAC_OUTER_DIAMETER = Units.inchesToMeters(237.31);
  public static final double TARMAC_FENDER_TO_TIP = Units.inchesToMeters(84.75);
  public static final double TARMAC_FULL_SIDE_LENGTH =
      TARMAC_INNER_DIAMETER * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
  public static final double TARMAC_MARKED_SIDE_LENGTH =
      Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
  public static final double TARMAC_MISSING_SIDE_LENGTH =
      TARMAC_FULL_SIDE_LENGTH - TARMAC_MARKED_SIDE_LENGTH; // Length removed b/c of corner cutoff
  public static final double HUB_SQUARE_LENGTH =
      TARMAC_OUTER_DIAMETER - (TARMAC_FENDER_TO_TIP * 2.0);

  // Reference rotations (angle from hub to each reference point and fender side)
  public static final Rotation2d REFERENCE_A_ROTATION =
      Rotation2d.fromDegrees(180.0)
          .minus(CENTER_LINE_ANGLE)
          .plus(Rotation2d.fromDegrees(360.0 / 16.0));
  public static final Rotation2d REFERENCE_B_ROTATION =
      REFERENCE_A_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d REFERENCE_C_ROTATION =
      REFERENCE_B_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d REFERENCE_D_ROTATION =
      REFERENCE_C_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d FENDER_A_ROTATION =
      REFERENCE_A_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 16.0));
  public static final Rotation2d FENDER_B_ROTATION =
      FENDER_A_ROTATION.rotateBy(Rotation2d.fromDegrees(90.0));

  // Reference points (centered of the sides of the tarmac if they formed a complete octagon, plus
  // edges of fender)
  public static final Pose2d REFERENCE_A =
      new Pose2d(HUB_CENTER, REFERENCE_A_ROTATION)
          .transformBy(GeomUtil.transformFromTranslation(TARMAC_INNER_DIAMETER / 2.0, 0.0));
  public static final Pose2d REFERENCE_B =
      new Pose2d(HUB_CENTER, REFERENCE_B_ROTATION)
          .transformBy(GeomUtil.transformFromTranslation(TARMAC_INNER_DIAMETER / 2.0, 0.0));
  public static final Pose2d REFERENCE_C =
      new Pose2d(HUB_CENTER, REFERENCE_C_ROTATION)
          .transformBy(GeomUtil.transformFromTranslation(TARMAC_INNER_DIAMETER / 2.0, 0.0));
  public static final Pose2d REFERENCE_D =
      new Pose2d(HUB_CENTER, REFERENCE_D_ROTATION)
          .transformBy(GeomUtil.transformFromTranslation(TARMAC_INNER_DIAMETER / 2.0, 0.0));
  public static final Pose2d FENDER_A =
      new Pose2d(HUB_CENTER, FENDER_A_ROTATION)
          .transformBy(GeomUtil.transformFromTranslation(HUB_SQUARE_LENGTH / 2.0, 0.0));
  public static final Pose2d FENDER_B =
      new Pose2d(HUB_CENTER, FENDER_B_ROTATION)
          .transformBy(GeomUtil.transformFromTranslation(HUB_SQUARE_LENGTH / 2.0, 0.0));

  // Cargo points
  public static final double CORNER_TO_CARGO_Y = Units.inchesToMeters(15.56);
  public static final double REFERENCE_TO_CARGO_Y =
      (TARMAC_FULL_SIDE_LENGTH / 2.0) - CORNER_TO_CARGO_Y;
  public static final double REFERENCE_TO_CARGO_X = Units.inchesToMeters(40.44);
  public static final Pose2d CARGO_A =
      REFERENCE_A.transformBy(
          GeomUtil.transformFromTranslation(REFERENCE_TO_CARGO_X, -REFERENCE_TO_CARGO_Y));
  public static final Pose2d CARGO_B =
      REFERENCE_A.transformBy(
          GeomUtil.transformFromTranslation(REFERENCE_TO_CARGO_X, REFERENCE_TO_CARGO_Y));
  public static final Pose2d CARGO_C =
      REFERENCE_B.transformBy(
          GeomUtil.transformFromTranslation(REFERENCE_TO_CARGO_X, REFERENCE_TO_CARGO_Y));
  public static final Pose2d CARGO_D =
      REFERENCE_C.transformBy(
          GeomUtil.transformFromTranslation(REFERENCE_TO_CARGO_X, -REFERENCE_TO_CARGO_Y));
  public static final Pose2d CARGO_E =
      REFERENCE_D.transformBy(
          GeomUtil.transformFromTranslation(REFERENCE_TO_CARGO_X, -REFERENCE_TO_CARGO_Y));
  public static final Pose2d CARGO_F =
      REFERENCE_D.transformBy(
          GeomUtil.transformFromTranslation(REFERENCE_TO_CARGO_X, REFERENCE_TO_CARGO_Y));

  // Terminal cargo point
  public static final Rotation2d TERMINAL_OUTER_ROTATION = Rotation2d.fromDegrees(133.75);
  public static final double TERMINAL_LENGTH = Units.inchesToMeters(324.0 - 256.42);
  public static final double TERMINAL_WIDTH =
      Math.tan(Rotation2d.fromDegrees(180.0).minus(TERMINAL_OUTER_ROTATION).getRadians())
          * TERMINAL_LENGTH;
  public static final Pose2d TERMINAL_CENTER =
      new Pose2d(
          new Translation2d(TERMINAL_LENGTH / 2.0, TERMINAL_WIDTH / 2.0),
          TERMINAL_OUTER_ROTATION.minus(Rotation2d.fromDegrees(90.0)));
  public static final double TERMINAL_CARGO_OFFSET = Units.inchesToMeters(10.43);
  public static final Pose2d CARGO_G =
      TERMINAL_CENTER.transformBy(GeomUtil.transformFromTranslation(TERMINAL_CARGO_OFFSET, 0.0));

  // Opposite reference points
  public static final Pose2d referenceAOpposite = opposite(REFERENCE_A);
  public static final Pose2d referenceBOpposite = opposite(REFERENCE_B);
  public static final Pose2d referenceCOpposite = opposite(REFERENCE_C);
  public static final Pose2d referenceDOpposite = opposite(REFERENCE_D);
  public static final Pose2d fenderAOpposite = opposite(FENDER_A);
  public static final Pose2d fenderBOpposite = opposite(FENDER_B);

  // Opposite cargo points
  public static final Pose2d cargoAOpposite = opposite(CARGO_A);
  public static final Pose2d cargoBOpposite = opposite(CARGO_B);
  public static final Pose2d cargoCOpposite = opposite(CARGO_C);
  public static final Pose2d cargoDOpposite = opposite(CARGO_D);
  public static final Pose2d cargoEOpposite = opposite(CARGO_E);
  public static final Pose2d cargoFOpposite = opposite(CARGO_F);
  public static final Pose2d cargoGOpposite = opposite(CARGO_G);

  // Calculate pose mirror on the opposite side of the field
  private static Pose2d opposite(Pose2d pose) {
    return new Pose2d(FIELD_LENGTH, FIELD_WIDTH, Rotation2d.fromDegrees(180.0))
        .transformBy(GeomUtil.poseToTransform(pose));
  }
}
