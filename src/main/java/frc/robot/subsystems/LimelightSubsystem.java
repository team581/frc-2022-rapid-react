// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable table;

  /** Creates a new LimelightSubsystem with the default NetworkTables table name ("limelight"). */
  public LimelightSubsystem() {
    this("limelight");
  }

  /** Creates a new LimelightSubsystem with the specified NetworkTables table name. */
  public LimelightSubsystem(String tableName) {
    table = NetworkTableInstance.getDefault().getTable(tableName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum LEDMode {
    /** Use the LED Mode set in the current pipeline. */
    CURRENT_PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private final int value;

    LEDMode(final int value) {
      this.value = value;
    }
  }

  public enum CamMode {
    VISION_PROCESSOR(0),
    DRIVER_CAMERA(1);

    private final int value;

    CamMode(final int value) {
      this.value = value;
    }
  }

  public enum StreamingMode {
    /** Side-by-side streams if a webcam is attached to Limelight. */
    STANDARD(0),
    /**
     * The secondary camera stream is placed in the lower-right corner of the primary camera stream.
     */
    PIP_MAIN(1),
    /**
     * The primary camera stream is placed in the lower-right corner of the secondary camera stream.
     */
    PIP_SECONDARY(2);

    private final int value;

    StreamingMode(final int value) {
      this.value = value;
    }
  }

  public enum SnapshotMode {
    OFF(0),
    /** Take two snapshots per second. */
    ON(1);

    private final int value;

    SnapshotMode(final int value) {
      this.value = value;
    }
  }

  /** Whether the Limelight has any valid targets. */
  public boolean hasTargets() {
    return table.getEntry("tv").getDouble(0) == 1;
  }

  /**
   * Horizontal offset from crosshair to target.
   *
   * <p><code>-24.85</code> to <code>24.85</code> degrees.
   */
  public double getX() {
    return table.getEntry("tx").getDouble(0);
  }

  /**
   * Vertical offset from crosshair to target.
   *
   * <p><code>-24.85</code> to <code>24.85</code> degrees.
   */
  public double getY() {
    return table.getEntry("ty").getDouble(0);
  }

  /** Target area (0% of image to 100% of image). */
  public double getArea() {
    return table.getEntry("ta").getDouble(0);
  }

  /** Skew or rotation (-90 degrees to 0 degrees). */
  public double getSkew() {
    return table.getEntry("ts").getDouble(0);
  }

  /**
   * The pipeline's latency contribution in milliseconds. Add at least 11ms for image capture
   * latency.
   */
  public double getPipelineLatency() {
    return table.getEntry("tl").getDouble(0);
  }

  /** Sidelength of shortest side of the fitted bounding box in pixels. */
  public double getShortestSide() {
    return table.getEntry("tshort").getDouble(0);
  }

  /**
   * Sidelength of longest side of the fitted bounding box in pixels.
   *
   * <p><code>0</code> to <code>320</code> pixels.
   */
  public double getLongestSide() {
    return table.getEntry("tlong").getDouble(0);
  }

  /**
   * Horizontal sidelength of the rough bounding box in pixels.
   *
   * <p><code>0</code> to <code>320</code> pixels.
   */
  public double getHorizontalSideLength() {
    return table.getEntry("thor").getDouble(0);
  }

  /**
   * Vertical sidelength of the rough bounding box in pixels.
   *
   * <p><code>0</code> to <code>320</code> pixels.
   */
  public double getVerticalSideLength() {
    return table.getEntry("tvert").getDouble(0);
  }

  /**
   * True active pipeline index of the camera.
   *
   * <p><code>0</code> to <code>9</code>.
   */
  public int getActivePipeline() {
    return (int) (table.getEntry("getpipe").getDouble(0));
  }

  /**
   * Results of a 3D position solution, 6 numbers: translation (x, y, z) and rotation (pitch, yaw,
   * roll).
   */
  public double[] getPosition3d() {
    // TODO: Check if this is actually a double array or if it's a number array
    // TODO: Create a 3D vector class
    return table.getEntry("pos3d").getDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
  }

  public LEDMode getLEDMode() {
    return LEDMode.values()[(int) table.getEntry("ledMode").getDouble(0)];
  }

  public void setLEDMode(final LEDMode mode) {
    table.getEntry("ledMode").setNumber(mode.value);
  }

  public CamMode getCamMode() {
    return CamMode.values()[(int) table.getEntry("camMode").getDouble(0)];
  }

  public void setCamMode(final CamMode mode) {
    table.getEntry("camMode").setNumber(mode.value);
  }

  /** Sets the camera's current pipeline. */
  public void setPipeline(final int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public StreamingMode getStreamingMode() {
    return StreamingMode.values()[(int) table.getEntry("stream").getDouble(0)];
  }

  public void setStreamingMode(final StreamingMode mode) {
    table.getEntry("stream").setNumber(mode.value);
  }

  public SnapshotMode getSnapshotMode() {
    return SnapshotMode.values()[(int) table.getEntry("snapshot").getDouble(0)];
  }

  public void setSnapshotMode(final SnapshotMode mode) {
    table.getEntry("snapshot").setNumber(mode.value);
  }
}
