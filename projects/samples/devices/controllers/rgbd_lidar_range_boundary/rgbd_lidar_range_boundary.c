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
      const int layer = layers / 2;

      const float *image =
        wb_lidar_get_layer_range_image(
          lidar,
          layer);

      const WbLidarPoint *points =
        wb_lidar_get_layer_point_cloud(
          lidar,
          layer);
      if (!image || !points) {
        fprintf(
          stderr,
          "[PUBLIC RGB POINT] ERROR: "
          "range image or point cloud is NULL\n");

        fflush(stderr);
        continue;
      }

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

      printf(
        "[PUBLIC RGB POINT] layer=%d\n",
        layer);

      for (int column = 124;
          column <= 131;
          ++column) {

        const WbLidarPoint *point =
          &points[column];

        printf(
          "[PUBLIC RGB POINT] "
          "column=%d "
          "XYZ=(%.6f,%.6f,%.6f) "
          "layer_id=%d "
          "RGBA=(%u,%u,%u,%u)\n",
          column,
          point->x,
          point->y,
          point->z,
          point->layer_id,
          (unsigned int)point->r,
          (unsigned int)point->g,
          (unsigned int)point->b,
          (unsigned int)point->a);
      }

      fflush(stdout);

      printed = 1;
    }
  }

  wb_robot_cleanup();

  return 0;
}
