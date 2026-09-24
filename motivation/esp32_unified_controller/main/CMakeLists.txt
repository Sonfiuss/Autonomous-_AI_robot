# The firmware's sources live in ../src and ../include so the tree is grouped by
# role (drivers, tasks) rather than by the IDF's component layout. This file is
# the only place that mapping is needed.
idf_component_register(
    SRCS "../src/main.cpp"
         "../src/context.cpp"
         "../src/motion_state.cpp"
         "../src/drivers/uart_link.cpp"
         "../src/drivers/step_dir_driver.cpp"
         "../src/drivers/servo_driver.cpp"
         "../src/tasks/comm_task.cpp"
         "../src/tasks/motion_task.cpp"
         "../src/tasks/peripheral_task.cpp"
         "../src/tasks/status_task.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES driver project)

# RM, MC and LINK are written to C++14.
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 14)
