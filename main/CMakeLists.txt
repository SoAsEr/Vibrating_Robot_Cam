
# install header file hierarchy
file(GLOB TAG_FILES ${PROJECT_SOURCE_DIR} apriltag/tag*.c)
set(APRILTAG_SRCS apriltag/apriltag.c apriltag/apriltag_pose.c apriltag/apriltag_quad_thresh.c)

file(GLOB COMMON_SRC apriltag/common/*.c)

idf_component_register(SRCS "main.c" ${TAG_FILES} ${APRILTAG_SRCS} ${COMMON_SRC}
                    INCLUDE_DIRS "." "apriltag" "apriltag/common")

set_source_files_properties(apriltag/common/matd.c PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)
