# ROS 2 Jazzy Installation Guide

This guide explains how to install **ROS 2 Jazzy** on **Ubuntu 24.04 (Noble)**.  
Follow the steps below to set up your environment properly.

---

## 🧩 Prerequisites

Make sure you have:

- **Ubuntu 24.04 (Noble Numbat)** (64-bit)
- **Sudo privileges**
- A stable internet connection

Update your system:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg2 lsb-release -y
```

---

## 🐍 1. Set Up Sources

Add the ROS 2 repository key:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository to your system:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

## ⚙️ 2. Install ROS 2 Jazzy

Update package index and install ROS 2 Jazzy Desktop:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop -y
```
---

## 🌐 3. Environment Setup

Source ROS 2 setup script:

```bash
source /opt/ros/jazzy/setup.bash
```

Make it permanent:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---
