cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Instalar los scripts de Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  rover_navigation/seguidor_dron.py
  rover_navigation/wander.py
  rover_navigation/seguidor_wander.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar otros directorios si existen
# install(
#   DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
