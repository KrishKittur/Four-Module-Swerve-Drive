// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Swerve Drive class stores the matrix which 
 * is used for inverse kinematics calculations
 */
public class SwerveDrive {
    private final SimpleMatrix matrix;

    /**
     * Constructs a new Swerve Drive class
     * 
     * @param fl the front left swerve module
     * @param fr the front right swerve module
     * @param bl the back left swerve module
     * @param br the back right swerve module
     */
    public SwerveDrive(SwerveModule fl, SwerveModule fr, SwerveModule bl, SwerveModule br) {
        double[][] data = {
            {1, 0, -fl.ryMeters}, 
            {0, 1, fl.rxMeters}, 
            {1, 0, -fr.ryMeters}, 
            {0, 1, fr.rxMeters},
            {1, 0, -bl.ryMeters},
            {0, 1, bl.rxMeters}, 
            {1, 0, -br.ryMeters},
            {0, 1, br.rxMeters}
        };
        matrix = new SimpleMatrix(data);
    }

    /**
     * Gets the module individual speeds according to the desired robot speeds 
     * 
     * @param xVelocityMetersPerSecond the desired x velocity of the robot (in meters per second)
     * @param yVelocityMetersPerSecond the desired y velocity of the robot (in meters per second)
     * @param omegaRadsPerSecond the desired angular velocity of the robot (in radians)
     * @return each swerve modules speed in an array of length 4, following the format: {fl, fr, bl, br}
     */
    public SwerveModuleState[] getSpeeds(double xVelocityMetersPerSecond, double yVelocityMetersPerSecond, double omegaRadsPerSecond) {
        double[][] ref = {{xVelocityMetersPerSecond}, {yVelocityMetersPerSecond}, {omegaRadsPerSecond}};
        SimpleMatrix components = matrix.mult(new SimpleMatrix(ref));
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i=0; i<7; i+=2) {
            double x = components.get(i, 0), y = components.get(i+1, 0);
            double velocity = Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
            double angle = Math.atan2(y,x);
            SwerveModuleState state =  new SwerveModuleState(velocity, angle);
            states[i/2] = state;
        }
        return states;
    }

    /**
     * Swerve Module class stores information 
     * about the swerve module's displacement 
     * from the center of the robot
     */
    public static class SwerveModule {
        public final double rxMeters, ryMeters;

        /**
         * Constructs a new swerve module with information 
         * about the modules displacement from the center 
         * of the robot
         * 
         * @param rxMeters the x displacement of the module to the center of the robot (in meters)
         * @param ryMeters the y displacement of the module to the center of the robot (in meters)
         */
        public SwerveModule(double rxMeters, double ryMeters) {
            this.rxMeters = rxMeters;
            this.ryMeters = ryMeters;
        }
    }

    /**
     * Swerve Module State class
     * contains module angles and speeds
     * to reach desired state
     */
    public static class SwerveModuleState {
        public final double velocityMetersPerSecond, angleRads;

        /**
         * Constructs a new module state to use as a reference for your module controllers 
         * 
         * @param velocityMetersPerSecond velocity of the module (in meters per second)
         * @param angleRads angle of the module (in radians)
         */
        public SwerveModuleState(double velocityMetersPerSecond, double angleRads) {
            this.velocityMetersPerSecond = velocityMetersPerSecond;
            this.angleRads = angleRads;
        }

        public String toString() {
            return String.format("Velocity: %.8fm/s, Angle (rads): %.8frads, Angle (degs): %.8fdegs", velocityMetersPerSecond, angleRads, Units.radiansToDegrees(angleRads));
        }
    }
}
