# Dream Team - T3001B - Foundation of Robotics Gpo 101

## 👥 Team

**Team:** Dream Team  
**Course:** T3001B - Foundation of Robotics Gpo 101

| Name | Matricula | Github User |
|------|-----------|-------------|
| Hector Tovar | A00840308 | @htovarm7 |
| José Luis Domínguez Morales  | A01285873 | @JLDominguezM |
| Paola Llamas Hernandez | A01178479 | @PaolaLlh18|
| Jocelyn Anahi Velarde Barrón | A01285780 | @JocelynVelarde |

## 🎯 Overview

This repository contains the coursework and implementations for the T3001B robotics course. The project demonstrates various robotics concepts including kinematics, control systems, and simulation for robotic arm manipulation.

## 📚 Project Modules

### Module 1 (M1) - Kinematics
**Teacher:** Luis Alberto Muñoz Ubando

- **Location:** `M1-Kinematics/`
- **Description:** Implementation of kinematics for robotic manipulators

### Module 4 (M4) - Robot Control
**Teacher:** Nezih Nieto Gutíerrez

- **Location:** `M4-RobotControl/`
- **Description:** Robot control implementation using simulation environment

## 🚀 Getting Started

### Prerequisites

- Docker installed on your system ([Get Docker](https://docs.docker.com/get-docker/))
- Git (for cloning the repository)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/JLDominguezM/dreamteam-T3001B.git
cd dreamteam-T3001B
```

2. Build the Docker image (see Docker Setup section below)

## 🐳 Docker Setup

This project includes a Dockerfile to simplify the evaluation process and ensure a consistent environment across different systems.

### Building the Docker Image

```bash
docker build -t dreamteam .
```

### Running the Docker Container

```bash
docker run -it --rm dreamteam
```

## 📁 Project Structure

```
dreamteam-T3001B/
├── README.md                          # This file
├── LICENSE                            # Project license
├── dockerfile                         # Docker configuration for easy evaluation
├── requirements.txt                         # Docker configuration for easy evaluation
│
├── M1-Kinematics/                     # Module 1: Kinematics
│   └── readme.md                      # Module-specific documentation
│
└── M4-RobotControl/                   # Module 4: Robot Control
    └── simulation_code/               # Actvity Robotics Control Lab 4.1
```

## 📝 For Teachers/Evaluators

This repository is structured to facilitate easy evaluation:

1. **Docker Support:** Use the provided Dockerfile to create a consistent evaluation environment
2. **Modular Organization:** Each module (M1, M4, etc.) is self-contained in its own directory

### Quick Evaluation Steps:

1. Build and run the Docker container
2. Navigate to the desired module directory

**Last Updated:** February 2026