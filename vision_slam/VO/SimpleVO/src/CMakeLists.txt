add_library( simplevo SHARED
    camera.cpp
)

target_link_libraries( simplevo
    ${THIRD_PARTY_LIBS}
)