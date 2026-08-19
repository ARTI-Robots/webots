#include <math.h>
#include <stdio.h>

#include <webots/lidar.h>
#include <webots/robot.h>

int main(void) {
  wb_robot_init();

  const int basic_time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag lidar = wb_robot_get_device("lidar");

  if (lidar == 0) {
    fprintf(stderr, "[OS0 TEST] ERROR: LiDAR device 'lidar' not found.\n");
    wb_robot_cleanup();
    return 1;
  }

  /*
   * 100 ms sampling period = 10 Hz.
   *
   * Point-cloud generation has to be enabled separately.
   */
  wb_lidar_enable(lidar, 100);
  wb_lidar_enable_point_cloud(lidar);

  const int width = wb_lidar_get_horizontal_resolution(lidar);
  const int layers = wb_lidar_get_number_of_layers(lidar);

  printf("[OS0 TEST] Controller started.\n");
  printf("[OS0 TEST] Basic timestep: %d ms\n", basic_time_step);
  printf("[OS0 TEST] LiDAR sampling period: 100 ms (10 Hz)\n");
  printf("[OS0 TEST] Horizontal resolution: %d\n", width);
  printf("[OS0 TEST] Layers: %d\n", layers);
  printf("[OS0 TEST] Total points per cloud: %d\n", width * layers);

  int steps = 0;
  int printed_cloud = 0;

  while (wb_robot_step(basic_time_step) != -1) {
    ++steps;

    /*
     * Give the sensor several simulation steps before inspecting
     * its first point cloud.
     */
    if (!printed_cloud && steps >= 10) {
      const WbLidarPoint *cloud = wb_lidar_get_point_cloud(lidar);

      if (!cloud) {
        printf("[OS0 TEST] Point cloud not available yet.\n");
        continue;
      }

      printf("[OS0 TEST] Point cloud received.\n");

      /*
       * Inspect four points around the middle vertical layer.
       * This is only a sanity check; with 128 layers there is
       * no single exact central layer.
       */
      const int layer = layers / 2;

      const int columns[4] = {
        0,
        width / 4,
        width / 2,
        3 * width / 4
      };

      for (int i = 0; i < 4; ++i) {
        const int column = columns[i];
        const int index = layer * width + column;

        const WbLidarPoint point = cloud[index];

        const double range =
          sqrt(point.x * point.x +
               point.y * point.y +
               point.z * point.z);

        printf(
          "[OS0 TEST] layer=%d col=%d "
          "XYZ=(%.4f, %.4f, %.4f) "
          "range=%.4f "
          "RGB=(%u,%u,%u)\n",
          layer,
          column,
          point.x,
          point.y,
          point.z,
          range,
          point.r,
          point.g,
          point.b
        );
      }

      printed_cloud = 1;
    }
  }

  wb_robot_cleanup();
  return 0;
}
