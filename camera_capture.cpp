#include <sl/Camera.hpp>
#include <iostream>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>

using namespace std;
using namespace sl;

int main(int argc, char** argv) {
    Camera zed;

    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::AUTO;
    init_params.camera_fps = 15;
    init_params.depth_mode = DEPTH_MODE::NEURAL_LIGHT;

    ERROR_CODE err = zed.open(init_params);
    if (err > ERROR_CODE::SUCCESS) {
        cout << "Error " << err << ", exit program." << endl;
        return EXIT_FAILURE;
    }

    std::filesystem::create_directories("camera_test");

    int i = 0;
    sl::Mat image;
    RuntimeParameters runtime_parameters;

    int64_t lastTime = 0;

    while (i < 1000) {
        if (zed.grab(runtime_parameters) <= ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image, VIEW::LEFT);

            Timestamp timestamp = zed.getTimestamp(TIME_REFERENCE::IMAGE);
            int64_t curTime = timestamp.getMilliseconds();

            cout << (curTime - lastTime) << endl;
            lastTime = curTime;

            string filename = "camera_test/" + to_string(curTime) + ".png";

            ERROR_CODE write_err = image.write(filename.c_str());
            if (write_err > ERROR_CODE::SUCCESS) {
                cout << "Failed to save: " << filename << endl;
            }

            i++;
        }
    }

    zed.close();
    return EXIT_SUCCESS;
}