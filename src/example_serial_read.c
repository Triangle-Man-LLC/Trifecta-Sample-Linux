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
fs_device_info_t imu_device;

/// @brief Initialize the Trifecta IMU device, and wait for connection to succeed.
/// @return 0 on success
int setup_imu(const char *port_name)
{
    int status = -1;

    // The following config values are recommended for Linux devices.
    fs_driver_config_t imu_config = FS_DRIVER_CONFIG_DEFAULT;
    fs_set_driver_parameters(&imu_device, &imu_config);

    // Wait for connection to complete...
    for (int i = 0; i < 5; i++)
    {
        // NOTE: Setting this value to -1 causes the driver to auto-detect the IMU device.
        // This is recommended in most cases, unless you have more than 1 device, in which case you should
        // enumerate each port
        status = fs_initialize_serial(&imu_device, (fs_serial_handle_t)port_name, FS_COMMUNICATION_MODE_USB_CDC);
        if (status == 0)
        {
            printf("Connected to Trifecta-K IMU %s (FD %ld)",
                   imu_device.device_descriptor.device_name,
                   imu_device.device_params.serial_port);
            break;
        }
        fs_closedown(&imu_device);
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
int main(int argc, char **argv)
{
    imu_device = FS_DEVICE_INFO_UNINITIALIZED;

    const char *port = NULL;
    if (argc >= 2)
        port = argv[1];
    
    if (!port)
    {
        printf("No port provided! Exiting...\n");
        return 0;
    }

    if (setup_imu(port) != 0)
    {
        printf("Failed to connect to IMU! Exiting...\n");
        return 0;
    }

    // Register signal handler for cleanup on termination
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    const int delay_time_ms = (5);

    uint32_t last_timestamp = 0;

    /// @brief Quaternion orientation of body relative to earth frame
    static fs_quaternion_t orientation_quaternion = {
        .w = 1,
        .x = 0,
        .y = 0,
        .z = 0,
    };

    /// @brief Euler orientation of body relative to earth frame
    static fs_vector3_t orientation_euler = {
        .x = 0,
        .y = 0,
        .z = 0,
    };

    // Turns on the IMU stream.
    fs_start_stream(&imu_device);

    // Read measurements in a loop. CTRL + C will exit the program.
    int counter = 0;
    while (1)
    {
        // It could be a good idea to send a stream keepalive signal to the IMU periodically.
        // To handle cases where it was disconnected for some reason.
        if (counter % 1000 == 0)
        {
            fs_start_stream(&imu_device);
        }
        counter++;

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

        usleep(delay_time_ms * 1000);
    }

    return 0;
}