// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.misc.SubsystemIO;
import java.util.ArrayList;
import java.util.List;
import lib.limelight.Limelight;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract interface VisionIO extends SubsystemIO<VisionIO.Inputs> {
  public class Inputs implements LoggableInputs {
    protected static List<Translation2d> coordinateArrayToTranslation2dList(
        double[] coordinateArray) {
      final List<Translation2d> corners = new ArrayList<>(coordinateArray.length / 2);

      for (int i = 0; i < coordinateArray.length; i += 2) {
        corners.add(new Translation2d(coordinateArray[i], coordinateArray[i + 1]));
      }

      return corners;
    }

    private static double[] translation2dListToCoordinateArray(List<Translation2d> corners) {
      final double[] coordinateArray = new double[corners.size() * 2];

      for (var i = 0; i < coordinateArray.length; i += 2) {
        final var corner = corners.get(i / 2);
        coordinateArray[i] = corner.getX();
        coordinateArray[i + 1] = corner.getY();
      }

      return coordinateArray;
    }

    public double captureTimestamp = 0;
    public List<Translation2d> corners = List.of();
    public boolean hasTargets = false;
    // TODO: Convert these values to radians
    /** Horizontal offset angle in degrees. */
    public double tx = 0;
    /** Vertical offset angle in degrees. */
    public double ty = 0;

    public void toLog(LogTable table) {
      table.put("CaptureTimestamp", captureTimestamp);
      table.put("HasTargets", hasTargets);
      table.put("Tx", tx);
      table.put("Ty", ty);
      table.put("Corners", translation2dListToCoordinateArray(corners));
    }

    public void fromLog(LogTable table) {
      captureTimestamp = table.getDouble("CaptureTimestamp", captureTimestamp);
      hasTargets = table.getBoolean("HasTargets", hasTargets);
      tx = table.getDouble("Tx", tx);
      ty = table.getDouble("Ty", ty);
      corners =
          coordinateArrayToTranslation2dList(
              table.getDoubleArray("Corners", translation2dListToCoordinateArray(corners)));
    }
  }

  public void setCamMode(Limelight.CamMode camMode);

  public void setStreamingMode(Limelight.StreamingMode streamingMode);

  public void setPipeline(int pipeline);
}
