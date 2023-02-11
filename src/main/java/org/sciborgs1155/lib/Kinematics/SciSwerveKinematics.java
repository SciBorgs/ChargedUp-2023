package org.sciborgs1155.lib.Kinematics;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;
import org.ejml.simple.SimpleMatrix;

/** 2nd Order Drive Kinematics for a Swerve Drive */
public class SciSwerveKinematics extends SwerveDriveKinematics {
  private final SimpleMatrix m_inverseKinematics;
  private final SimpleMatrix m_forwardKinematics;

  private final int m_numModules;
  private final Translation2d[] m_modules;
  private SciSwerveModuleState[] m_moduleStates;
  private Translation2d m_prevCoR = new Translation2d();

  public SciSwerveKinematics(Translation2d... wheelsMeters) {

    if (wheelsMeters.length < 2) {
      throw new IllegalArgumentException("A swerve drive requires at least two modules");
    }
    m_numModules = wheelsMeters.length;
    m_modules = Arrays.copyOf(wheelsMeters, m_numModules);
    m_moduleStates = new SciSwerveModuleState[m_numModules];
    Arrays.fill(m_moduleStates, new SwerveModuleState());
    m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

    for (int i = 0; i < m_numModules; i++) {
      m_inverseKinematics.setRow(
          i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getX(), -m_modules[i].getY());
      m_inverseKinematics.setRow(
          i * 2 + 1, 0, /* Start Data */ 0, 1, -m_modules[i].getY(), +m_modules[i].getX());
    }
    m_forwardKinematics = m_inverseKinematics.pseudoInverse();

    MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
  }

  public SciSwerveModuleState[] toSwerveModuleStates(
      ChassisState chassisState, Translation2d centerOfRotationMeters) {
    if (chassisState.axMetersPerSecondSquared == 0
        && chassisState.ayMetersPerSecondSquared == 0
        && chassisState.alphaRadiansPerSecondSquared == 0) {
      SciSwerveModuleState[] newModuleStates = new SciSwerveModuleState[m_numModules];
      for (int i = 0; i < m_numModules; i++) {
        newModuleStates[i] = new SciSwerveModuleState(0.0, 0.0, m_moduleStates[i].angle);
      }
      m_moduleStates = newModuleStates;
      return m_moduleStates;
    }
    if (!centerOfRotationMeters.equals(m_prevCoR)) {
      for (int i = 0; i < m_numModules; i++) {
        m_inverseKinematics.setRow(
            i * 2 + 0,
            0, /* Start Data */
            1,
            0,
            -m_modules[i].getY() + centerOfRotationMeters.getY());
        m_inverseKinematics.setRow(
            i * 2 + 1,
            0, /* Start Data */
            0,
            1,
            +m_modules[i].getX() - centerOfRotationMeters.getX());
      }
      m_prevCoR = centerOfRotationMeters;
    }
    var robotStateVector = new SimpleMatrix(4, 1);
    robotStateVector.setColumn(
        0,
        0,
        chassisState.axMetersPerSecondSquared,
        chassisState.ayMetersPerSecondSquared,
        chassisState.omegaRadiansPerSecond * chassisState.omegaRadiansPerSecond,
        chassisState.alphaRadiansPerSecondSquared);
    var moduleStatesMatrix = m_inverseKinematics.mult(robotStateVector);

    m_moduleStates = new SciSwerveModuleState[m_numModules];
    for (int i = 0; i < m_numModules; i++) {
      double x = moduleStatesMatrix.get(i * 2, 0);
      double y = moduleStatesMatrix.get(i * 2 + 1, 0);

      double acceleration = Math.hypot(x, y);
      Rotation2d angle = new Rotation2d(x, y);

      m_moduleStates[i] = new SciSwerveModuleState(acceleration, angle);
    }

    return m_moduleStates;
  }

  public SciSwerveModuleState[] toSwerveModuleStates(ChassisState chassisSpeeds) {
    return toSwerveModuleStates(chassisSpeeds, new Translation2d());
  }
}
