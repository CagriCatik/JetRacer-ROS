# Jetracer-ROS

This project is a ROS-based application for using the JetRacer with support for various setup scripts and comprehensive documentation.

## Introduction

Jetracer-ROS is a project based on NVIDIA's **JetRacer**, enabling integration with the Robot Operating System (ROS). It includes installation scripts, documentation, and various setup instructions to facilitate the setup and use of the JetRacer with ROS.

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/your-username/jetracer-ros.git
   ```

2. Run the setup scripts in the `setup` folder to install the required dependencies:
   - `install-etcher.sh` for installing Etcher
   - `install-nomachine.sh` for setting up NoMachine for remote connections
   - `install-ros.sh` for installing ROS

   Example:

   ```bash
   ./setup/install-ros.sh
   ```

3. Follow the instructions in the [Documentation](#documentation) to complete the remaining steps.

## Usage

After successful installation, you can start ROS and use the JetRacer with the created ROS nodes.

```bash
roslaunch jetracer jetracer.launch
```

Further usage instructions can be found in the documentation.
Here’s an updated section for the README, specifically for mdBook-based documentation:

## Documentation

The documentation is built using [mdBook](https://rust-lang.github.io/mdBook/), and is located in the `docs` directory.

You can build and serve the documentation locally by following these steps:

1. Install `mdBook`:

   ```bash
   cargo install mdbook
   ```

2. Navigate to the `docs` directory and build the book:

   ```bash
   cd docs
   mdbook serve
   ```

3. Open your browser and navigate to `http://localhost:3000` to view the documentation.

For more details on contributing to the documentation or generating new content, refer to the `book.toml` configuration file.
You can browse the documentation directly in the project or use the `book.toml` file to create a book format.

## Setup Scripts

In the `setup` folder, you will find various scripts that help with the project setup:

- `install_etcher.sh`: Installs Etcher for flashing images.
- `install-nomachine.sh`: Installs and sets up NoMachine for remote desktop connections.
- `install-ros.sh`: Installs ROS on the JetRacer.

Run the scripts in sequence to fully set up the environment.
