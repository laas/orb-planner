# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME)
  ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} KineoController)

  # Link against package library.
  TARGET_LINK_LIBRARIES(${NAME} 
    ${Boost_LIBRARIES}
    ${PROJECT_NAME})
ENDMACRO(ADD_TESTCASE)

# Generated test.
ADD_TESTCASE(plan)
KINEO_STANDALONE(plan)

ADD_TESTCASE(plan-test)
KINEO_STANDALONE(plan-test)