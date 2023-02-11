
package org.sciborgs1155.lib.Kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisState extends ChassisSpeeds{
    /** Represents forward acceleration w.r.t the robot frame of reference. (Fwd is +) */
    public double axMetersPerSecondSquared;
  
    /** Represents sideways velocity w.r.t the robot frame of reference. (Left is +) */
    public double ayMetersPerSecondSquared;
  
    /** Represents the angular velocity of the robot frame. (CCW is +) */
    public double alphaRadiansPerSecondSquared;
  
    /** Constructs a ChassisState with zeros for dx, dy, and theta. */
    public ChassisState() {}
  
    /**
     * Constructs a ChassisSpeeds object.
     *
     * @param axMetersPerSecondSquared Forward velocity.
     * @param ayMetersPerSecondSquared Sideways velocity.
     * @param alphaRadiansPerSecondSquared Angular velocity.
     */
    public ChassisState(
        double axMetersPerSecond, double ayMetersPerSecond, double alphaRadiansPerSecondSquared) {
      super(0,0,0);
      this.axMetersPerSecondSquared = axMetersPerSecond;
      this.ayMetersPerSecondSquared = ayMetersPerSecond;
      this.alphaRadiansPerSecondSquared = alphaRadiansPerSecondSquared;
    }
    public ChassisState(
        double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond,
        double axMetersPerSecond, double ayMetersPerSecond, double alphaRadiansPerSecondSquared) {
      super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
      this.axMetersPerSecondSquared = axMetersPerSecond;
      this.ayMetersPerSecondSquared = ayMetersPerSecond;
      this.alphaRadiansPerSecondSquared = alphaRadiansPerSecondSquared;
    }
  

    public static ChassisState fromFieldRelativeSpeeds(
        double axMetersPerSecondSquared,
        double ayMetersPerSecondSquared,
        double alphaRadiansPerSecondSquared,
        Rotation2d robotAngle) {
      return new ChassisState(
          axMetersPerSecondSquared * robotAngle.getCos() + ayMetersPerSecondSquared * robotAngle.getSin(),
          -axMetersPerSecondSquared * robotAngle.getSin() + ayMetersPerSecondSquared * robotAngle.getCos(),
          alphaRadiansPerSecondSquared);
    }
  
    
    public static ChassisState fromFieldRelativeSpeeds(
        ChassisState fieldRelativeSpeeds, Rotation2d robotAngle) {
      return fromFieldRelativeSpeeds(
          fieldRelativeSpeeds.axMetersPerSecondSquared,
          fieldRelativeSpeeds.ayMetersPerSecondSquared,
          fieldRelativeSpeeds.alphaRadiansPerSecondSquared,
          robotAngle);
    }
  
    @Override
    public String toString() {
      return String.format(
          "ChassisSpeeds(Ax: %.2f m/s, Ay: %.2f m/s, Alpha: %.2f rad/s)",
          axMetersPerSecondSquared, ayMetersPerSecondSquared, alphaRadiansPerSecondSquared);
    }
  }