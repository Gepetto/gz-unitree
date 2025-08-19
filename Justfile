sdf_path := "models/h1_2_with_ground.sdf"
dds_config := "<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
camera_pose := "position { x: 1.9028059244155884, y: -2.756911039352417, z: 2.2905828952789307 }, orientation { x: -0.23666168749332428, y: 0.19653673470020294, z: 0.73200297355651855, w: 0.607895016670227 }"


sim logfile="with_imu_on_model.log":
    echo Dont forget to add plugin filename="gz-unitree" name="gz::unitree::UnitreePlugin" to your SDF file, in the model tag
    just install && LD_LIBRARY_PATH="/usr/local/lib/gz-unitree:/usr/local/lib/gz-video-recorder${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" CYCLONEDDS_URI="{{dds_config}}" GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/gz-unitree/ GZ_GUI_PLUGIN_PATH=/usr/local/lib/gz-video-recorder gz sim --headless-rendering {{sdf_path}} -v 4 

setup-test:
    #!/usr/bin/env bash
    if ! [ -d h1v2-Isaac ]; then 
        git clone https://github.com/gwennlbh/h1v2-Isaac h1v2-Isaac -b devel
        cd h1v2-Isaac
        uv sync
        cd ..
    fi
    if ! [ -d unitree_sdk2_python ]; then 
        git clone https://github.com/unitreerobotics/unitree_sdk2_python
    fi

test:
    #!/usr/bin/env bash
    just setup-test
    cd h1v2-Isaac
    gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: { {{ camera_pose }} }"
    gz service -s /gui/record_video/start --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --req 'data: "mp4"'
    gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'pause: false'
    CYCLONEDDS_URI="{{dds_config}}" uv run --active deploy/sim2real.py || true
    gz service -s /gui/record_video/stop --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --req 'data: "file:///tmp/result.mp4"'


setup:
    #!/usr/bin/env bash
    set -euxo pipefail

    if [[ -f /etc/arch-release ]]; then
        if ! command -v paru &> /dev/null; then
            echo "paru is not installed. Please install paru."
            exit 1
        fi

        paru -S gz-harmonic gz-physics=7 gz-cmake3 gz-plugin2 gz-common5 sdformat=14 urdfdom gz-sensors8 gz-sim8

    elif [[ -f /etc/debian_version ]] && [[ ! $(command -v gz) ]]; then
        sudo apt-get update
        sudo apt-get install curl lsb-release gnupg
        sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
        sudo apt-get update
        sudo apt-get install gz-harmonic
    fi

    cd unitree_sdk2
    mkdir build; cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
    sudo make install

install:
    #!/usr/bin/env bash
    mkdir -p build
    cd build
    CMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake:/usr/lib/cmake cmake ..
    sudo mkdir -p /usr/local/lib/gz-unitree/
    make && sudo cp libgz-unitree.so /usr/local/lib/gz-unitree/

video_recorder_plugin:
    #!/usr/bin/env bash
    cd gz_video_recorder
    if [[ -f /etc/debian_version ]]; then
        sudo apt install --yes libgz-gui9-dev
    fi
    mkdir -p build
    cd build
    cmake ..
    sudo mkdir -p /usr/local/lib/gz-video-recorder/
    make && sudo cp libgz-video-recorder.so /usr/local/lib/gz-video-recorder/
