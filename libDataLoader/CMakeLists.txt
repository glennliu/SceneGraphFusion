SET(targetname libDataLoader)

INCLUDE(UseOpenCV)
INCLUDE(UseEigen3)

SET(INSTALL_OPTION ON)

SET(sources
        src/dataloader_scannet.cpp
        src/dataloader_3rscan.cpp
        src/dataloader_scenenn.cpp
        ScanNetLoader/sensorData.cpp
)
SET(headers
        include/dataLoader/dataset_loader.h
        include/dataLoader/dataset_loader_facotry.h
)

ADD_LIBRARY(${targetname} ${sources} ${headers})
TARGET_INCLUDE_DIRECTORIES(${targetname} PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ScanNetLoader>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/ScanNetLoader>
)

INCLUDE(LinkEigen3)
INCLUDE(LinkOpenCV)
INCLUDE(LinkJson)

############ INSTALL ############

if(INSTALL_OPTION)
        install(TARGETS ${targetname}
                EXPORT "${targetname}Targets"
                LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
                ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

        install(TARGETS libJson EXPORT ${targetname}Targets)

        install(EXPORT "${targetname}Targets"
                FILE "${targetname}Targets.cmake"
                DESTINATION cmake
        )

        include(CMakePackageConfigHelpers)

        # create config file
        configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
                "${CMAKE_CURRENT_BINARY_DIR}/${targetname}Config.cmake"
                INSTALL_DESTINATION cmake
        )

        # install config files
        install(FILES
                "${CMAKE_CURRENT_BINARY_DIR}/${targetname}Config.cmake"
                # "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
                DESTINATION cmake
        )

        install(DIRECTORY include/dataLoader DESTINATION include)
        install(FILES ScanNetLoader/sensorData.h DESTINATION include/ScanNetLoader)

        # generate the export targets for the build tree
        # export(EXPORT "${targetname}Targets"
        # FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/${targetname}Targets.cmake"
        # )

endif()