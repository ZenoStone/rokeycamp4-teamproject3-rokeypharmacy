cmake_minimum_required(VERSION 3.8)
project(pharmacy_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 추가 ------------------------------------------
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rosidl_default_generators : 코드 생성기( .msg .srv .action 을 각 언어별로 코드로 자동 생성해주는 빌드 도구)
#                             .msg, .srv, .action 파일을 읽어서 c++ python 등 언어 타입지원 코드를 생성시키고
#                             colconbuild시에 msg 파일을 코드로 변환하는 역학을 하며 자동실행됨
# builtin_interfaces : ros2 인터페이스 메시지에서 공통으로 사용되는 기본타입을 정의한 패키지
#                             Time, Duration, String, Bool, Int32, Float64 등등



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



# ROS2 입문 2차시 p24 내용 추가
set(msg_files

)

set(srv_files
  "srv/MediCine.srv"
  "srv/MediInfo.srv"
)

set(action_files

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)
# ROS2 입문 2차시 p24 내용 추가 끝 --------------------



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
