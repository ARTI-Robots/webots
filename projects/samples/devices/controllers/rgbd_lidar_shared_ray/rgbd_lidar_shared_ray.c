#include <stdio.h>

#include <webots/lidar.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();

  const int time_step =
    (int)wb_robot_get_basic_time_step();

  WbDeviceTag lidar =
    wb_robot_get_device("lidar");

  wb_lidar_enable(lidar, time_step);

  int step_count = 0;
  bool printed = false;

  while (wb_robot_step(time_step) != -1) {
    ++step_count;

    if (!printed && step_count >= 2) {
      const int resolution =
        wb_lidar_get_horizontal_resolution(lidar);

      const int layers =
        wb_lidar_get_number_of_layers(lidar);

      const int layer =
        layers / 2;

      const int column =
        resolution / 2;

      const float *image =
        wb_lidar_get_layer_range_image(
          lidar,
          layer);

      printf(
        "[RGB-LIDAR TEST] "
        "layer=%d column=%d range=%.6f\n",
        layer,
        column,
        image[column]);

      fflush(stdout);
      printed = true;
    }
  }

  wb_robot_cleanup();
  return 0;
}