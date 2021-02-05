add_library( simplevo SHARED
    camera.cpp
    frame.cpp
    mappoint.cpp
    map.cpp
    config.cpp
)

target_link_libraries( simplevo
    ${THIRD_PARTY_LIBS}
)