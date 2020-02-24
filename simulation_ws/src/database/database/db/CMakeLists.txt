cmake_minimum_required(VERSION 2.8.3)
project(database)

add_subdirectory(db)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/db/test.db"
        DESTINATION "${CMAKE_BINARY_DIR}/db"
)



