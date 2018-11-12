list(APPEND astra_libs libMWClosestPoint.so libOpenNI2.so libOniFile.so liborbbec.so libPSLink.so)

list(APPEND dep_libs gomp moped ${OpenCV_LIBS} ${GLEW_LIBRARIES} ${IL_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

set(EXE_LISTS_ASTRA main_astra.cpp UBT_multiObjectPosEstimation.cpp astra_src/astra.cpp)
add_executable(demo_astra ${EXE_LISTS_ASTRA})
set_target_properties(demo_astra PROPERTIES COMPILE_FLAGS "-O3 -std=gnu++11 -pthread -Wno-deprecated-declarations -march=native -mfpmath=sse -fopenmp")
target_link_libraries(demo_astra ${astra_libs} ${dep_libs})




