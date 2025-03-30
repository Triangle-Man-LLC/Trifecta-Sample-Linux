///
/// In this example, data from the Trifecta device is sent over serial.
/// The code in this file is released into the public domain.
///

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>

#include "../lib/Trifecta-Driver/FS_Trifecta.h"

static const char *TAG = "IMU_SERIAL_TEST";

/// @brief IMU device handle
static fs_device_info imu_device = FS_DEVICE_INFO_UNINITIALIZED;

/// @brief Initialize the Trifecta IMU device, and wait for connection to succeed.
/// @return 0 on success
int setup_imu()
{
    int status = -1;

    // The following config values are recommended for resource constrained Linux devices such as Raspberry Pi 3.
    fs_driver_config imu_config;
    imu_config.background_task_core_affinity = -1;
    imu_config.background_task_priority = -1;
    imu_config.read_timeout_micros = 5000;
    imu_config.task_wait_ms = 2;
    imu_config.task_stack_size_bytes = 128 * 1024;

    // Wait for connection to complete...
    for (int i = 0; i < 50; i++)
    {
        // NOTE: Setting this value to -1 causes the driver to auto-detect the IMU device.
        fs_set_driver_parameters(&imu_device, &imu_config);
        // This is recommended in most cases, unless you have specifically configured it.
        // If that is the case, then you need to assign serial_fd by calling open() on that specific serial port name.
        int serial_fd = -1;
        status = fs_initialize_serial(&imu_device, serial_fd);
        if (status == 0)
        {
            break;
        }
        printf("Waiting for IMU connection!\n");
        usleep(10000);
    }

    return status;
}

/// @brief Signal handler to ensure fs_closedown() is called on shutdown
void handle_signal(int signal)
{
    printf("Received signal %d, closing down...\n", signal);
    fs_closedown(&imu_device);
    exit(signal);
}

/// @brief Demonstrates the setup and read of Trifecta IMU using serial connection.
/// The IMU is made to read the data once. You can do this in a loop if you would like.
/// @return
int main()
{
    if (setup_imu() != 0)
    {
        printf("Failed to connect to IMU! Exiting...\n");
        exit(0);
    }

    // Register signal handler for cleanup on termination
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    const int delay_time_ms = (5);

    uint32_t last_timestamp = 0;

    /// @brief Quaternion orientation of body relative to earth frame
    static fs_quaternion orientation_quaternion = {
        .w = 1,
        .x = 0,
        .y = 0,
        .z = 0,
    };

    /// @brief Euler orientation of body relative to earth frame
    static fs_vector3 orientation_euler = {
        .x = 0,
        .y = 0,
        .z = 0,
    };

    // Read measurements in a loop. CTRL + C will exit the program.
    while (1)
    {
        fs_read_one_shot(&imu_device); // Request a single read from the IMU.

        usleep(delay_time_ms * 1000);

        if (fs_get_last_timestamp(&imu_device, &last_timestamp) != 0)
        {
            printf("Failed to get packet time stamp!\n");
        }

        // Orientation (quaternion) and orientation (euler) are 2 of the possible outputs that you can get from the IMU.
        // Please look at the functions in FS_Trifecta.h to see which other ones are available.
        if (fs_get_orientation(&imu_device, &orientation_quaternion) != 0)
        {
            printf("Did not receive orientation quaternion update for some reason!\n");
        }
        if (fs_get_orientation_euler(&imu_device, &orientation_euler, true) != 0)
        {
            printf("Did not receive orientation euler update for some reason!\n");
        }

        // NOTE: Logging is somewhat CPU intensive, and you should consider turning it off except for debugging purposes.
        printf("Timestamp (%u)\nQuaternion (%.6f, %.6f, %.6f, %.6f)\nEuler (%.2f, %.2f, %.2f)\n", last_timestamp,
               orientation_quaternion.w, orientation_quaternion.x, orientation_quaternion.y,
               orientation_quaternion.z, orientation_euler.x, orientation_euler.y, orientation_euler.z);
    }

    return 0;
}