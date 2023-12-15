#!/usr/bin/env python3

from px4_custom_interfaces.srv import MergeTarget
import rclpy
from rclpy.node import Node
import os
import pandas as pd


class DataMergerService(Node):

    def __init__(self):
        super().__init__('data_merger_service')
        self.srv = self.create_service(MergeTarget, 'merge_target', self.merger_callback)

    def merger_callback(self, request, response):
        response.success = False

        csv_files = [file for file in os.listdir(request.directory.data) if file.endswith('.csv')]
        if len(csv_files) < 2:
            self.get_logger().error("Insufficient CSV files for inner join.")
            return response

        # Load the first CSV file
        df_result = pd.read_csv(os.path.join(request.directory.data, csv_files[0]))
        initial = os.path.splitext(csv_files[0])[0]  # Extract the base name without the extension
        initial = f"_{initial}"

        first_run = True
        # Iterate through the remaining CSV files and perform inner join
        for csv_file in csv_files[1:]:
            df_current = pd.read_csv(os.path.join(request.directory.data, csv_file))
            if first_run:
                first_run = False
            else:
                initial = ''
            current = os.path.splitext(csv_file)[0]  # Extract the base name without the extension
            df_result = pd.merge(df_result, df_current, how='outer', on='timestamp',
                                suffixes=(f'{initial}', f'_{current}'))

        df_result.sort_values(by='timestamp', inplace=True)
        # df_result.interpolate(method='linear', inplace=True)

        # # Fill NaN values at the edges
        # df_result.ffill(inplace=True)  # Forward fill
        # df_result.bfill(inplace=True)  # Backward fill
        # Specify the output CSV file path
        output_csv_path = f'{request.directory.data}/merged.csv'

        # Write the result DataFrame to the output CSV file
        df_result.to_csv(output_csv_path, index=False)

        response.success = True
        return response


def main(args=None) -> None:
    print('Starting data_merger_service node...')
    rclpy.init(args=args)
    try:
        data_merger_service = DataMergerService()
        rclpy.spin(data_merger_service)
    except KeyboardInterrupt:
        pass
    try:
        rclpy.shutdown()
    except Exception as e:
        print(e)
        pass


if __name__ == '__main__':
    main()

