#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import pandas as pd
from std_msgs.msg import String


class DataMergerSubscriber(Node):
    def __init__(self):
        super().__init__("data_merger_subscriber")
        self.subscriber = self.create_subscription(String, "merge_target_directory", self.directory_callback, 10)
        self.forbidden_list = ["clip_counter", "temperature", "timestamp_sample", "samples", "error_count"]

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

        # Finally merge the faults with the sensor datas
        first = full_merge['timestamp'][0]
        full_merge.iloc[0] = 0
        full_merge.at[0, 'timestamp'] = first
        full_merge.ffill(inplace=True)
        full_merge = pd.merge(df_result, full_merge, on='timestamp', how='outer')
        full_merge.sort_values(by='timestamp', inplace=True)

        self.drop_columns(full_merge)

        if preempted:
            out_file_name = "merged_preempted.csv"
        else:
            out_file_name = "merged.csv"

        # Specify the output CSV file path
        output_csv_path = f"{directory_path}/{out_file_name}"

        # Write the result DataFrame to the output CSV file
        full_merge.to_csv(output_csv_path, index=False)

        self.get_logger().info(f"Data merged successfully into {output_csv_path}")

    def populate_inactive_faults(input_df):
        output_df = input_df
        return output_df

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
