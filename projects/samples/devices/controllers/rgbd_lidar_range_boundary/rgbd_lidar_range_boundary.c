#include <stdio.h>

#include <webots/lidar.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();

  const int time_step =
    (int)wb_robot_get_basic_time_step();

  WbDeviceTag lidar =
    wb_robot_get_device("lidar");

  wb_lidar_enable(
    lidar,
    time_step);

  wb_lidar_enable_point_cloud(lidar);

  int step_count = 0;
  int printed = 0;

  while (wb_robot_step(time_step) != -1) {
    ++step_count;

    if (!printed && step_count >= 2) {
      const int layers =
        wb_lidar_get_number_of_layers(lidar);

      // For 6 layers this is layer 3.
      // Earlier diagnostics showed layer 3 maps
      // to merged panorama row 5.
      const int layer =
        layers / 2;

      const float *image =
        wb_lidar_get_layer_range_image(
          lidar,
          layer);

      printf(
        "[PUBLIC BOUNDARY] layer=%d\n",
        layer);

      for (int column = 124;
           column <= 131;
           ++column) {
        printf(
          "[PUBLIC BOUNDARY] "
          "column=%d range=%.6f\n",
          column,
          image[column]);
      }

      fflush(stdout);

      printed = 1;
    }
  }

  wb_robot_cleanup();

  return 0;
}
