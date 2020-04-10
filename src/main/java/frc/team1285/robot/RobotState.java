package frc.team1285.robot;

import java.util.Map;

import frc.team1285.util.kinematics.InterpolatingDouble;
import frc.team1285.util.kinematics.InterpolatingTreeMap;
import frc.team254.lib.geometry.Pose2d;

public class RobotState {
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    private static final int kObservationBufferSize = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;

    private double distance_driven_;

    private RobotState() {
        reset(0, new Pose2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);

        distance_driven_ = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

}