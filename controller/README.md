# ESP Eye Controller

This is the softare for an ESP32-based controller for the eye


## Hardware Setup

- **Board:** ESP32-WROOM-32 (AZDelivery NodeMCU USB-C)
- **Motor Pins:**
  - MOTOR1_PWM_A_PIN: GPIO 32
  - MOTOR1_PWM_B_PIN: GPIO 33
  - MOTOR2_PWM_A_PIN: GPIO 25
  - MOTOR2_PWM_B_PIN: GPIO 26
- **I2C (IMU):**
  - SDA: GPIO 21
  - SCL: GPIO 22
- **Hall Sensor:**
  - GPIO 4 (currently disabled in code)
- **Power:**
  - Use onboard USB-C for power and programming.
- **Boot:**
  - Standard ESP32-WROOM-32 boot sequence.

## Setup

- install esp-idf
- configure dev env for dev board


## Development

- due to ESP-IDF expecting project config to be in root folder, be sure to work from 'controller/' dir as root folder (file -> open folder)
- or use file -> add folder to workspace -> add `controller/`, then select `controller/` as active ESP-IDF folder (toolbar bottom left)



## Usage

    idf.py build
    idf.py flash



# _Sample project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)



## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
