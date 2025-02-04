# rasp_ros_iot
Projeto utilizando ROS2 e IoT, com a placa raspberry pi pico w.

## 1 - Instalação
Instale o ROS2

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Instale o micro-ros
1 - Dependências:
- tinyxml2
```bash
cd ~/
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2
mkdir build
cd build
sudo cmake ..
sudo make
sudo make install
```
- python3-rosdep:
```bash
sudo apt install python3-rosdep
```

2 - Crie o workspace:
```bash
mkdir ~/uros_ws && cd ~/uros_ws
mkdir src
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo rosdep init
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
echo "source ~/uros_ws/install/local_setup.sh" >> ~/.bashrc
source ~/.bashrc
```

3 - Instale o micro agent:
```bash
sudo apt update
sudo apt install snapd
sudo snap install micro-ros-agent
sudo snap set core experimental.hotplug=true
sudo systemctl restart snapd
snap connect micro-ros-agent:serial-port snapd:pico
```

## Como rodar:
1 - Clone o repositório:
```bash
git clone https://github.com/henriqueguanais/rasp_ros_iot.git
```

2 - Importe a pasta rasp_ros_iot, que está dentro do repositório, como projeto na extensão Raspberry Pi Pico Project

3 - Inicialize o micro agent:
```bash
micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200
```

- Caso não funcione:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4
```

4 - Compile e run no projeto

- O endereço IP aparecerá no display OLED.
- Para visualizar o conteúdo postado no tópico:
```bash
ros2 topic echo /pico_publisher 
```



