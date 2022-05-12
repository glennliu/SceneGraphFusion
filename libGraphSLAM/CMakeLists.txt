SET(targetname libGraphSLAM)

INCLUDE(UseEigen3)
INCLUDE(UseOpenCV)

SET(INSTALL_OPTION ON)

if(APPLE)
    set(INSEG_LIB ${PROJECT_SOURCE_DIR}/lib/libInSegLib_osx.a)
else()
    set(INSEG_LIB ${PROJECT_SOURCE_DIR}/lib/libInSegLib_linux.a)
endif()

FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.h")
IF(BUILD_GRAPHPRED)
    INCLUDE(UseOnnxRuntime)
    FILE(GLOB src_gp "graphPredictor/*.cpp")
    FILE(GLOB h_gp   "graphPredictor/*.h")
    SET(sources ${sources} ${src_gp})
    SET(headers ${headers} ${h_gp})
ENDIF(BUILD_GRAPHPRED)



ADD_LIBRARY(${targetname} ${sources} ${headers})
TARGET_LINK_LIBRARIES(${targetname}
        PUBLIC  ${INSEG_LIB}
)
TARGET_INCLUDE_DIRECTORIES(${targetname} 
    PUBLIC 
        ${CMAKE_SOURCE_DIR}/lib
        ${CMAKE_SOURCE_DIR}/ORUtils/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/graphPredictor>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<INSTALL_INTERFACE:include/${targetname}>
        # $<INSTALL_INTERFACE:${CMAKE_SOURCE_DIR}/lib>
)

IF(BUILD_GRAPHPRED)
    INCLUDE(LinkOnnxRuntime)
    target_compile_definitions(${targetname} PUBLIC COMPILE_WITH_GRAPHPRED)
ENDIF(BUILD_GRAPHPRED)
INCLUDE(Linktinyply)
INCLUDE(LinkJson)

############# INSTALL ###########
if(INSTALL_OPTION)
    install(TARGETS ${targetname}
        EXPORT "${targetname}Targets"
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    install(TARGETS libtinyply EXPORT ${targetname}Targets)

    install(EXPORT "${targetname}Targets"
        FILE "${targetname}Targets.cmake"
        DESTINATION cmake
    )

    include(CMakePackageConfigHelpers)

    configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/${targetname}Config.cmake"
        INSTALL_DESTINATION cmake
    )

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${targetname}Config.cmake"
        DESTINATION cmake
    )

    install(FILES ${headers} DESTINATION include/${targetname})
endif()