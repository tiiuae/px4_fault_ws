import argparse
import yaml


def parse_arguments():
    parser = argparse.ArgumentParser(description="Edit YAML configuration for drone simulation.")

    # Boundaries
    parser.add_argument("--ns_lower", type=float, help="North-south lower boundary")
    parser.add_argument("--ns_upper", type=float, help="North-south upper boundary")
    parser.add_argument("--ew_lower", type=float, help="East-west lower boundary")
    parser.add_argument("--ew_upper", type=float, help="East-west upper boundary")
    parser.add_argument("--alt_lower", type=float, help="Altitude lower boundary")
    parser.add_argument("--alt_upper", type=float, help="Altitude upper boundary")
    parser.add_argument("--yaw_lower", type=float, help="Yaw lower boundary")
    parser.add_argument("--yaw_upper", type=float, help="Yaw upper boundary")

    # Waypoints
    parser.add_argument("--wp_lower", type=int, help="Lower limit for waypoints")
    parser.add_argument("--wp_upper", type=int, help="Upper limit for waypoints")

    # Iterations
    parser.add_argument("--iterations", type=int, help="Number of iterations")

    # Durations
    parser.add_argument("--duration_faults", nargs=2, type=float, help="Duration for faults [start, end]")
    parser.add_argument("--duration_between_faults", nargs=2, type=float, help="Duration between faults [start, end]")
    parser.add_argument("--time_to_fault_start", nargs=2, type=float, help="Time to fault start [start, end]")

    # Sensor::Accelerometer
    parser.add_argument("--sensor_accel_record", type=bool, help="To record accelerometer outputs [true/false]")
    parser.add_argument("--sensor_accel_gaussian_activate", type=bool, help="Activate Gaussian noise [true/false]")
    parser.add_argument("--sensor_accel_gaussian", nargs=2, type=float, help="Standard deviation of normal distribution [lower, upper]")
    parser.add_argument("--sensor_accel_shift_activate", type=bool, help="Activate sensor shift [true/false]")
    parser.add_argument("--sensor_accel_shift", nargs=2, type=float, help="Percentage (written as decimal) of shift in actual value [lower, upper]")
    parser.add_argument("--sensor_accel_scale_activate", type=bool, help="Activate sensor scale [true/false]")
    parser.add_argument("--sensor_accel_scale", nargs=2, type=float, help="Scalar multiplier to scale actual value [lower, upper]")
    parser.add_argument("--sensor_accel_drift_activate", type=bool, help="Activate sensor drift [true/false]")
    parser.add_argument("--sensor_accel_drift", nargs=2, type=float, help="Percentage (as percent) of increase w.r.t. original value with each second [lower, upper]")

    # Sensor::Gyroscope
    parser.add_argument("--sensor_gyro_record", type=bool, help="To record gyroscope outputs [true/false]",)
    parser.add_argument("--sensor_gyro_gaussian_activate", type=bool, help="Activate Gaussian noise [true/false]")
    parser.add_argument("--sensor_gyro_gaussian", nargs=2, type=float, help="Standard deviation of normal distribution [lower, upper]")
    parser.add_argument("--sensor_gyro_shift_activate", type=bool, help="Activate sensor shift [true/false]")
    parser.add_argument("--sensor_gyro_shift", nargs=2, type=float, help="Percentage (written as decimal) of shift in actual value [lower, upper]")
    parser.add_argument("--sensor_gyro_scale_activate", type=bool, help="Activate sensor scale [true/false]")
    parser.add_argument("--sensor_gyro_scale", nargs=2, type=float, help="Scalar multiplier to scale actual value [lower, upper]")
    parser.add_argument("--sensor_gyro_drift_activate", type=bool, help="Activate sensor drift [true/false]")
    parser.add_argument("--sensor_gyro_drift", nargs=2, type=float, help="Percentage (as percent) of increase w.r.t. original value with each second [lower, upper]")

    # Sensor::Magnetometer
    parser.add_argument("--sensor_mag_record", type=bool, help="To record magnetometer outputs [true/false]")
    parser.add_argument("--sensor_mag_gaussian_activate", type=bool, help="Activate Gaussian noise [true/false]")
    parser.add_argument("--sensor_mag_gaussian", nargs=2, type=float, help="Standard deviation of normal distribution [lower, upper]")
    parser.add_argument("--sensor_mag_shift_activate", type=bool, help="Activate sensor shift [true/false]")
    parser.add_argument("--sensor_mag_shift", nargs=2, type=float, help="Percentage (written as decimal) of shift in actual value [lower, upper]")
    parser.add_argument("--sensor_mag_scale_activate", type=bool, help="Activate sensor scale [true/false]")
    parser.add_argument("--sensor_mag_scale", nargs=2, type=float, help="Scalar multiplier to scale actual value [lower, upper]")
    parser.add_argument("--sensor_mag_drift_activate", type=bool, help="Activate sensor drift [true/false]")
    parser.add_argument("--sensor_mag_drift", nargs=2, type=float, help="Percentage (as percent) of increase w.r.t. original value with each second [lower, upper]")

    # Sensor::Barometer
    parser.add_argument("--sensor_baro_record", type=bool, help="To record barometer outputs [true/false]")
    parser.add_argument("--sensor_baro_gaussian_activate", type=bool, help="Activate Gaussian noise [true/false]")
    parser.add_argument("--sensor_baro_gaussian", nargs=2, type=float, help="Standard deviation of normal distribution [lower, upper]")
    parser.add_argument("--sensor_baro_shift_activate", type=bool, help="Activate sensor shift [true/false]")
    parser.add_argument("--sensor_baro_shift", nargs=2, type=float, help="Percentage (written as decimal) of shift in actual value [lower, upper]")
    parser.add_argument("--sensor_baro_scale_activate", type=bool, help="Activate sensor scale [true/false]")
    parser.add_argument("--sensor_baro_scale", nargs=2, type=float, help="Scalar multiplier to scale actual value [lower, upper]")
    parser.add_argument("--sensor_baro_drift_activate", type=bool, help="Activate sensor drift [true/false]")
    parser.add_argument("--sensor_baro_drift", nargs=2, type=float, help="Percentage (as percent) of increase w.r.t. original value with each second [lower, upper]")

    return parser.parse_args()


def update_config(data, args):
    # Update boundaries if arguments are provided
    if args.ns_lower is not None:
        data["boundaries"]["north_south"]["lower"] = args.ns_lower
    if args.ns_upper is not None:
        data["boundaries"]["north_south"]["upper"] = args.ns_upper
    if args.ew_lower is not None:
        data["boundaries"]["east_west"]["lower"] = args.ew_lower
    if args.ew_upper is not None:
        data["boundaries"]["east_west"]["upper"] = args.ew_upper
    if args.alt_lower is not None:
        data["boundaries"]["altitude"]["lower"] = args.alt_lower
    if args.alt_upper is not None:
        data["boundaries"]["altitude"]["upper"] = args.alt_upper
    if args.yaw_lower is not None:
        data["boundaries"]["yaw"]["lower"] = args.yaw_lower
    if args.yaw_upper is not None:
        data["boundaries"]["yaw"]["upper"] = args.yaw_upper

    # Update waypoints if arguments are provided
    if args.wp_lower is not None:
        data["number_of_waypoints"]["lower"] = args.wp_lower
    if args.wp_upper is not None:
        data["number_of_waypoints"]["upper"] = args.wp_upper

    # Update iterations
    if args.iterations is not None:
        data["iterations"] = args.iterations

    # Update durations
    if args.duration_faults is not None:
        data["duration_for_faults"] = args.duration_faults
    if args.duration_between_faults is not None:
        data["duration_between_faults"] = args.duration_between_faults
    if args.time_to_fault_start is not None:
        data["time_to_fault_start"] = args.time_to_fault_start

    # Update sensor configurations
    sensor_mappings = {
        "accelerometer": {
            "record": args.sensor_accel_record,
            "gaussian_noise._NOISE": {
                "active": args.sensor_accel_gaussian_activate,
                "vals": args.sensor_accel_gaussian,
            },
            "bias_shift._SHIF": {
                "active": args.sensor_accel_shift_activate,
                "vals": args.sensor_accel_shift,
            },
            "bias_scale._SCAL": {
                "active": args.sensor_accel_scale_activate,
                "vals": args.sensor_accel_scale,
            },
            "drift._DRIFT": {
                "active": args.sensor_accel_drift_activate,
                "vals": args.sensor_accel_drift,
            },
        },
        "gyroscope": {
            "record": args.sensor_gyro_record,
            "gaussian_noise._NOISE": {
                "active": args.sensor_gyro_gaussian_activate,
                "vals": args.sensor_gyro_gaussian,
            },
            "bias_shift._SHIF": {
                "active": args.sensor_gyro_shift_activate,
                "vals": args.sensor_gyro_shift,
            },
            "bias_scale._SCAL": {
                "active": args.sensor_gyro_scale_activate,
                "vals": args.sensor_gyro_scale,
            },
            "drift._DRIFT": {
                "active": args.sensor_gyro_drift_activate,
                "vals": args.sensor_gyro_drift,
            },
        },
        "magnetometer": {
            "record": args.sensor_mag_record,
            "gaussian_noise._NOISE": {
                "active": args.sensor_mag_gaussian_activate,
                "vals": args.sensor_mag_gaussian,
            },
            "bias_shift._SHIF": {
                "active": args.sensor_mag_shift_activate,
                "vals": args.sensor_mag_shift,
            },
            "bias_scale._SCAL": {
                "active": args.sensor_mag_scale_activate,
                "vals": args.sensor_mag_scale,
            },
            "drift._DRIFT": {
                "active": args.sensor_mag_drift_activate,
                "vals": args.sensor_mag_drift,
            },
        },
        "barometer": {
            "record": args.sensor_baro_record,
            "gaussian_noise._NOISE": {
                "active": args.sensor_baro_gaussian_activate,
                "vals": args.sensor_baro_gaussian,
            },
            "bias_shift._SHIF": {
                "active": args.sensor_baro_shift_activate,
                "vals": args.sensor_baro_shift,
            },
            "bias_scale._SCAL": {
                "active": args.sensor_baro_scale_activate,
                "vals": args.sensor_baro_scale,
            },
            "drift._DRIFT": {
                "active": args.sensor_baro_drift_activate,
                "vals": args.sensor_baro_drift,
            },
        },
    }

    for sensor in data["sensors"]:
        module_name = sensor["module_name"]
        if module_name in sensor_mappings:
            sensor_config = sensor_mappings[module_name]
            # Update 'record' if present
            if "record" in sensor_config and sensor_config["record"] is not None:
                sensor["record"] = sensor_config["record"]
            for fault in sensor["faults"]:
                fault_type = fault["type"]
                if fault_type in sensor_config:
                    fault_config = sensor_config[fault_type]
                    # Update 'active' if present
                    if "active" in fault_config and fault_config["active"] is not None:
                        fault["active"] = fault_config["active"]
                    # Update 'vals' if present
                    if "vals" in fault_config and fault_config["vals"] is not None:
                        fault["vals"] = fault_config["vals"]

    return data


def main():
    args = parse_arguments()

    with open("./src/px4_fault_injection/config/circuit_params.yaml", "r") as file:
        data = yaml.safe_load(file)

    data = update_config(data, args)
    print(data)
    with open(
        "./install/px4_fault_injection/share/px4_fault_injection/config/circuit_params.yaml",
        "w",
    ) as file:
        yaml.safe_dump(data, file, default_flow_style=False)

    print("\nConfiguration updated!")


if __name__ == "__main__":
    main()
