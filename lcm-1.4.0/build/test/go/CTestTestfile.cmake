# CMake generated Testfile for 
# Source directory: /home/zakhar/work/SI/lcm-1.4.0/test/go
# Build directory: /home/zakhar/work/SI/lcm-1.4.0/build/test/go
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(Go::client_server "/usr/bin/cmake" "-E" "env" "GOPATH=/home/zakhar/work/SI/lcm-1.4.0/build/test/types/go:" "/usr/bin/python3.10" "/home/zakhar/work/SI/lcm-1.4.0/test/go/../run_client_server_test.py" "/home/zakhar/work/SI/lcm-1.4.0/build/test/c/test-c-server" "/usr/bin/go" "test" "/home/zakhar/work/SI/lcm-1.4.0/test/go/client_test.go")
set_tests_properties(Go::client_server PROPERTIES  _BACKTRACE_TRIPLES "/home/zakhar/work/SI/lcm-1.4.0/test/go/CMakeLists.txt;4;add_test;/home/zakhar/work/SI/lcm-1.4.0/test/go/CMakeLists.txt;0;")
add_test(Go::unit_test "/usr/bin/go" "test" "-v" "./...")
set_tests_properties(Go::unit_test PROPERTIES  WORKING_DIRECTORY "/home/zakhar/work/SI/lcm-1.4.0/test/go/../../lcm-go/" _BACKTRACE_TRIPLES "/home/zakhar/work/SI/lcm-1.4.0/test/go/CMakeLists.txt;13;add_test;/home/zakhar/work/SI/lcm-1.4.0/test/go/CMakeLists.txt;0;")
