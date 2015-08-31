cmake_minimum_required(VERSION 2.8.3)
project(table_object)

# set(Boost_DIR /home/zengzhen/Documents/boost_1_56_0)
# set(Boost_ADDITIONAL_VERSIONS "1.56" "1.56.0")
# set(Boost_NO_SYSTEM_PATHS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  pcl_conversions
  ros_sec
  roscpp
  sensor_msgs
  tf
  message_generation
  std_msgs
  geometry_msgs
  cmake_modules
)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen REQUIRED)


message( "\${PCL_INCLUDE_DIRS} = ${PCL_INCLUDE_DIRS}" )
message( "\${PCL_LIBRARIES} = ${PCL_LIBRARIES}" )
# message("\${BOOST_LIBRARIES} = ${BOOST_LIBRARIES}")

set(OpenCV_INCLUDE_DIRS /usr/include;/usr/include/opencv)
set(OpenCV_LIBS /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so /usr/lib/x86_64-linux-gnu/libopencv_contrib.so /usr/lib/x86_64-linux-gnu/libopencv_core.so /usr/lib/x86_64-linux-gnu/libopencv_features2d.so /usr/lib/x86_64-linux-gnu/libopencv_flann.so /usr/lib/x86_64-linux-gnu/libopencv_gpu.so /usr/lib/x86_64-linux-gnu/libopencv_highgui.so /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so /usr/lib/x86_64-linux-gnu/libopencv_legacy.so /usr/lib/x86_64-linux-gnu/libopencv_ml.so /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so /usr/lib/x86_64-linux-gnu/libopencv_photo.so /usr/lib/x86_64-linux-gnu/libopencv_stitching.so /usr/lib/x86_64-linux-gnu/libopencv_ts.so /usr/lib/x86_64-linux-gnu/libopencv_video.so /usr/lib/x86_64-linux-gnu/libopencv_videostab.so)

set(MLPACK_INCLUDE_DIRS /usr/local/include;/usr/include/libxml2)
set(MLPACK_BOOST_LIB_DIR /usr/lib/x86_64-linux-gnu)
set(MLPACK_LIBS /usr/local/lib/libmlpack.so; general;${MLPACK_BOOST_LIB_DIR}/libboost_program_options.so;general;
${MLPACK_BOOST_LIB_DIR}/libboost_unit_test_framework.so;general; ${MLPACK_BOOST_LIB_DIR}/libboost_random.so;
/usr/lib/x86_64-linux-gnu/libxml2.so;/usr/lib/libarmadillo.so;general;rt)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  bottle_feature.msg
  palm_reflex_triggered.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  record_feature.srv
  detect_touch.srv
  palm_reflex.srv
  detect_touch_update.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  moveit_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#     INCLUDE_DIRS include
	LIBRARIES table_object
	CATKIN_DEPENDS moveit_msgs pcl_conversions ros_sec roscpp sensor_msgs tf
	DEPENDS system_lib message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
	/home/zengzhen/Documents/tools
	/home/zengzhen/Documents/ros_zhen/src/baxter_moveit/include
	/home/zengzhen/Documents/ros_ws/devel/include
	${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${MLPACK_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
add_definitions(${EIGEN_DEFINITIONS})

## Declare a cpp library
# add_library(table_object
#   src/${PROJECT_NAME}/table_object.cpp
# )
add_library(table_obj_exp src/table_obj_exp.cpp)

## Declare a cpp executable
add_executable(table_bottle_publisher src/table_bottle_publisher.cpp)
add_executable(table_bottle_publisher_v0 src/table_bottle_publisher_v0.cpp)
add_executable(feature_extraction src/feature_extraction.cpp)
add_executable(tabletop_initializer src/tabletop_initializer.cpp)
add_executable (alignment_prerejective src/alignment_prerejective.cpp)
add_executable (align_bottle src/align_bottle.cpp)
add_executable (tracking_sample src/tracking_sample.cpp)
add_executable(robot_scene_tracking src/robot_scene_tracking.cpp)
add_executable(my_tf_listener src/my_tf_listener.cpp)
add_executable(my_tf_broadcaster src/my_tf_broadcaster.cpp)
add_executable(record_feature_server src/record_feature_server.cpp)
add_executable(record_feature_client src/record_feature_client.cpp)
add_executable(learn_action src/learn_action.cpp)
add_executable(touch_detector_server src/touch_detector_server.cpp)
add_executable(touch_detector_client src/touch_detector_client.cpp)
add_executable(palm_reflex_server src/palm_reflex_server.cpp)
add_executable(close_gripper src/close_gripper.cpp)

add_executable(action_learning src/action_learning.cpp)
add_executable(test_learning src/test_learning.cpp)

add_executable(action_learning_update src/action_learning_update.cpp)
add_executable(touch_detector_server_update src/touch_detector_server_update.cpp)

add_executable(rand_action src/rand_action.cpp)
add_executable(inv_palm_reflex_server src/inv_palm_reflex_server.cpp)

add_executable(save_cluster src/save_cluster.cpp)
add_executable(cylinder_block_tracking src/cylinder_block_tracking.cpp)

add_executable(pcl_visualizer_demo src/pcl_visualization_demo.cpp)

add_executable(octree_change_detection src/octree_change_detection.cpp)

add_executable(example_normal_estimation src/example_normal_estimation.cpp)

add_executable(testExp src/testExp.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(table_object_node table_object_generate_messages_cpp)
add_dependencies(record_feature_server ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(record_feature_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(robot_scene_tracking ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(touch_detector_server ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(touch_detector_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(palm_reflex_server ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(action_learning ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(test_learning ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(action_learning_update ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(touch_detector_server_update ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(inv_palm_reflex_server ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(save_cluster ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(table_bottle_publisher
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(table_bottle_publisher_v0
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(feature_extraction
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtable_obj_seg.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libpcd_cloud.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtrackRigid.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtrack3D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview2D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview3D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libutil.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libcolorDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtouchDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libmoveDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libbottleDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libmainGraph.so
)

target_link_libraries(tabletop_initializer
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtable_obj_seg.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libpcd_cloud.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtrackRigid.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview2D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview3D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libutil.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libcolorDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtouchDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libmoveDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libmainGraph.so
#     /home/zengzhen/Documents/tools/Kinect/lib/libconvert_rgbd_to_image.so
)

target_link_libraries (alignment_prerejective ${PCL_LIBRARIES})

target_link_libraries (align_bottle ${PCL_LIBRARIES})

target_link_libraries (tracking_sample ${PCL_LIBRARIES})

target_link_libraries(robot_scene_tracking
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtable_obj_seg.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libpcd_cloud.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtrackRigid.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtrack3D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview2D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview3D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libutil.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libcolorDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtouchDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libmoveDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libbottleDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libmainGraph.so
#     /home/zengzhen/Documents/tools/Kinect/lib/libconvert_rgbd_to_image.so
)

target_link_libraries(my_tf_listener ${catkin_LIBRARIES})

target_link_libraries(my_tf_broadcaster ${catkin_LIBRARIES})

target_link_libraries(record_feature_server ${catkin_LIBRARIES})

target_link_libraries(record_feature_client ${catkin_LIBRARIES})

target_link_libraries(learn_action ${catkin_LIBRARIES} ${MLPACK_LIBS} table_obj_exp)

target_link_libraries(touch_detector_server ${catkin_LIBRARIES})

target_link_libraries(touch_detector_client ${catkin_LIBRARIES})

target_link_libraries(palm_reflex_server ${catkin_LIBRARIES})

target_link_libraries(close_gripper ${catkin_LIBRARIES})

target_link_libraries(action_learning ${catkin_LIBRARIES} table_obj_exp)

target_link_libraries(test_learning ${catkin_LIBRARIES} table_obj_exp)

target_link_libraries(action_learning_update ${catkin_LIBRARIES} table_obj_exp)

target_link_libraries(touch_detector_server_update ${catkin_LIBRARIES})

target_link_libraries(rand_action ${catkin_LIBRARIES})

target_link_libraries(inv_palm_reflex_server ${catkin_LIBRARIES})

target_link_libraries(save_cluster
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtable_obj_seg.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libpcd_cloud.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview2D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview3D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libutil.so
#     /home/zengzhen/Documents/tools/Kinect/lib/libconvert_rgbd_to_image.so
)

target_link_libraries(cylinder_block_tracking
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtable_obj_seg.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libpcd_cloud.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtrack3D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtrackModel.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview2D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libview3D.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libutil.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libcolorDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libtouchDetector.so
    /home/zengzhen/Documents/ros_sec_ws/devel/lib/libbottleDetector.so
#     /home/zengzhen/Documents/tools/Kinect/lib/libconvert_rgbd_to_image.so
)

target_link_libraries (
    pcl_visualizer_demo 
    ${PCL_LIBRARIES}
)

target_link_libraries(
    octree_change_detection
    ${PCL_LIBRARIES}
)

target_link_libraries(
    example_normal_estimation
    ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS table_object table_object_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_table_object.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
