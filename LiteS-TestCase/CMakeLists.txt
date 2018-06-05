FILE(GLOB subdirs *)

FOREACH(child ${subdirs})
    IF(IS_DIRECTORY ${child})
        STRING(REGEX MATCH T-.*$ project  ${child})
        ADD_SUBDIRECTORY(${project})
    ENDIF()
ENDFOREACH()

