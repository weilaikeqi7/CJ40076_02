if(NOT DEFINED OUTPUT)
    message(FATAL_ERROR "OUTPUT is required")
endif()

string(TIMESTAMP APP_BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")

file(WRITE "${OUTPUT}"
"#ifndef BUILD_INFO_H
#define BUILD_INFO_H

#define APP_BUILD_TIMESTAMP \"${APP_BUILD_TIMESTAMP}\"

#endif /* BUILD_INFO_H */
")
