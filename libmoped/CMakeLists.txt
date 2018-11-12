find_package(GLEW REQUIRED)
find_package(DevIL REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/libmoped/libs/siftfast
	${PROJECT_SOURCE_DIR}/libmoped/libs/ann_1.1.1
	${PROJECT_SOURCE_DIR}/libmoped/libs/ann_1.1.1/ANN
	${PROJECT_SOURCE_DIR}/libmoped/libs/levmar-2.4
	${PROJECT_SOURCE_DIR}/libmoped/libs/SiftGPU
	${PROJECT_SOURCE_DIR}/libmoped/include
	${PROJECT_SOURCE_DIR}/libmoped/src
)

include_directories(
  ${OpenGL_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIRS}
  ${IL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR}
  ${GL_INCLUDE_DIRS}
)

file(GLOB_RECURSE ANN_LIST ${PROJECT_SOURCE_DIR}/libmoped/libs/ann_1.1.1/*.cpp)
add_library(ann SHARED ${ANN_LIST})
set_target_properties(ann PROPERTIES COMPILE_FLAGS "-w -O3 -march=native -mfpmath=sse -fopenmp -funroll-loops -ffast-math -fno-trapping-math -fsingle-precision-constant")

set(LM_LIST ${PROJECT_SOURCE_DIR}/libmoped/libs/levmar-2.4/lmbc.c
	    ${PROJECT_SOURCE_DIR}/libmoped/libs/levmar-2.4/lmblec.c
	    ${PROJECT_SOURCE_DIR}/libmoped/libs/levmar-2.4/misc.c
            ${PROJECT_SOURCE_DIR}/libmoped/libs/levmar-2.4/Axb.c
            ${PROJECT_SOURCE_DIR}/libmoped/libs/levmar-2.4/lm.c
            ${PROJECT_SOURCE_DIR}/libmoped/libs/levmar-2.4/lmlec.c)
add_library(lm SHARED ${LM_LIST})
set_target_properties(lm PROPERTIES COMPILE_FLAGS "-w -O3 -march=native -mfpmath=sse -fopenmp -funroll-loops -ffast-math -fno-trapping-math -fsingle-precision-constant")

file(GLOB SIFTFAST_LIST ${PROJECT_SOURCE_DIR}/libmoped/libs/siftfast/*.cpp)
add_library(siftfast SHARED ${SIFTFAST_LIST})
set_target_properties(siftfast PROPERTIES COMPILE_FLAGS "-w -O3 -march=native -mfpmath=sse -fopenmp -funroll-loops -ffast-math -fno-trapping-math -fsingle-precision-constant")

file(GLOB SIFTGPU_LIST ${PROJECT_SOURCE_DIR}/libmoped/libs/SiftGPU/*.cpp)
add_library(SiftGPU SHARED ${SIFTGPU_LIST})
set_target_properties(SiftGPU PROPERTIES COMPILE_FLAGS "-w -O3 -march=native -mfpmath=sse -fopenmp -funroll-loops -ffast-math -fno-trapping-math -fsingle-precision-constant -DSIFTGPU_NO_CG")
target_link_libraries(SiftGPU ${GLEW_LIBRARIES} ${IL_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

set(LIB_DEPS siftfast ann lm SiftGPU)

add_library(moped SHARED ${PROJECT_SOURCE_DIR}/libmoped/src/moped.cpp)
set_target_properties(moped PROPERTIES COMPILE_FLAGS "-w -O3 -march=native -mfpmath=sse -funroll-loops -fopenmp -ffast-math -fno-trapping-math")
target_link_libraries(moped gomp
  ${LIB_DEPS} 
  ${OpenCV_LIBS}
  ${GLEW_LIBRARIES}
  ${IL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )
add_dependencies(moped ${LIB_DEPS})
