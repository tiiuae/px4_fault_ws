#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import pandas as pd
from std_msgs.msg import String
import yaml
from ament_index_python.packages import get_package_share_directory


class DataMergerSubscriber(Node):
    def __init__(self):
        super().__init__("data_merger_subscriber")
        self.subscriber = self.create_subscription(String, "merge_target_directory", self.directory_callback, 10)

        self.mission_params = None
        config_path = get_package_share_directory(
            "px4_fault_injection") + "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        self.metadata = None
        config_path = get_package_share_directory(
            "px4_fault_injection") + "/config/metadata.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.metadata = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        self.fault_label_descriptors = None
        config_path = get_package_share_directory(
            "px4_fault_injection") + "/config/fault_label_descriptors.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.fault_label_descriptors = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        self.forbidden_list = ["clip_counter", "temperature", "timestamp_sample", "samples", "error_count"]
        self.ignore_list = ["timestamp", "device_id"]

    def directory_callback(self, msg):
        directory_path = msg.data
        csv_files = [file for file in os.listdir(directory_path) if file.endswith(".csv")]

        if len(csv_files) < 2:
            self.get_logger().error("Insufficient CSV files for merging.")
            return

        # Load the first CSV file
        df_result = pd.read_csv(os.path.join(directory_path, csv_files[0]))
        # Iterate through the remaining CSV files and perform inner join
        for csv_file in csv_files[1:]:
            # First only merge the sensor recordings
            if csv_file == 'faults.csv':
                continue

            df_current = pd.read_csv(os.path.join(directory_path, csv_file))
            df_result = pd.merge(df_result, df_current, how="outer", on="timestamp")

        df_result.sort_values(by="timestamp", inplace=True)

        # Append the faults now
        faults_df = pd.read_csv(os.path.join(directory_path, 'faults.csv'))
        if faults_df['timestamp'][faults_df.tail(1).index.start] == "PREEMPTED":
            preempted = True
            faults_df = faults_df.iloc[:-1]
            faults_df['timestamp'] = faults_df['timestamp'].astype(int)
        else:
            preempted = False
        # Populate timestaps into faults
        timestamps = df_result[["timestamp"]]
        full_merge = pd.merge(timestamps, faults_df, on='timestamp', how='outer')
        full_merge.sort_values(by='timestamp', inplace=True)
        full_merge = self.populate_inactive_faults(full_merge)

        # Finally merge the faults with the sensor datas
        first = full_merge['timestamp'][0]
        full_merge.iloc[0] = 0
        full_merge.at[0, 'timestamp'] = first
        full_merge.ffill(inplace=True)
        full_merge = pd.merge(df_result, full_merge, on='timestamp', how='outer')
        full_merge.sort_values(by='timestamp', inplace=True)

        full_merge = self.drop_columns(full_merge)
        full_merge = self.rename_headers(full_merge)

        if preempted:
            out_file_name = "merged_preempted.csv"
        else:
            out_file_name = "merged.csv"

        # Specify the output CSV file path
        output_csv_path = f"{directory_path}/{out_file_name}"

        # Write the result DataFrame to the output CSV file
        full_merge.to_csv(output_csv_path, index=False)

        self.get_logger().info(f"Data merged successfully into {output_csv_path}")

    def rename_headers(self, input_df):
        if self.metadata is None or self.fault_label_descriptors is None:
            self.get_logger().error("Label descriptor files are missing.")
            return

        for column in input_df.columns:
            try:
                heading, name = column.split("|")
                for sensor in self.metadata['sensors']:
                    if sensor['name'] == name and heading not in self.ignore_list:
                        input_df = input_df.rename(columns={
                                                   column: f"{sensor['struct'][heading]['description']}|{sensor['struct'][heading]['unit']}|{name}"
                                                   })
            except ValueError:
                pass
            try:
                item, name, fault_type = column.split("_")
                if item == "SENS":
                    for sensor in self.fault_label_descriptors['sensors']:
                        if name == sensor['label']:
                            for fault in sensor['type']:
                                if fault_type == fault['label']:
                                    input_df = input_df.rename(columns={
                                        column: f"{fault['description']}|{sensor['name']}"
                                    })
            except ValueError:
                pass
        return input_df

    def populate_inactive_faults(self, input_df):
        if self.mission_params is None:
            self.get_logger().error("Mission parameter file missing.")
            return

        for sensor in self.mission_params['sensors']:
            root = sensor["root"]
            for fault_type in sensor["faults"]:
                fault_type, label = fault_type['type'].split('.')
                if root + label not in input_df.columns:
                    input_df[root + label] = 0.0
                else:
                    continue
        return input_df

    def drop_columns(self, input_df):
        for header in list(input_df.columns):
            for drop in self.forbidden_list:
                if drop in header:
                    input_df.drop(header, axis = 1, inplace=True)
        return input_df


def main(args=None):
    print("Starting data_merger_subscriber node...")
    rclpy.init(args=args)
    data_merger_subscriber = DataMergerSubscriber()

    try:
        rclpy.spin(data_merger_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
