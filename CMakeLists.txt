add_library(3rd::crypto STATIC IMPORTED GLOBAL) 
if(WIN32)
    set_target_properties(3rd::crypto PROPERTIES
        INTERFACE_LINK_DIRECTORIES IMPORTED_LINK_INTERFACE_LIBRARIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/curl/lib/libcrypto.lib
    )
endif()

add_library(3rd::ssl STATIC IMPORTED GLOBAL) 
if(WIN32)
    set_target_properties(3rd::ssl PROPERTIES
        INTERFACE_LINK_DIRECTORIES IMPORTED_LINK_INTERFACE_LIBRARIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/curl/lib/libssl.lib
    )
endif()

add_library(3rd::curl STATIC IMPORTED GLOBAL)
if(WIN32)
    set_target_properties(3rd::curl PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/curl/include/win"
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/curl/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/curl/lib/libcurld.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/curl/lib/libcurl.lib

        IMPORTED_LINK_INTERFACE_LIBRARIES "3rd::crypto;3rd::ssl"
    )
elseif(UNIX)
    set_target_properties(3rd::curl PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/curl/include/linux"
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/curl/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/curl/lib/libcurl.a

        IMPORTED_LINK_INTERFACE_LIBRARIES "z;crypto;ssl"
    )
else()
    message(STATUS "unsupport os")
endif()

# gmock
add_library(3rd::gmock STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::gmock PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/gmock"
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib/gmockd.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib/gmock.lib
    )
elseif(UNIX)
    set_target_properties(3rd::gmock PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/gmock"
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib/libgmockd.a
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib/libgmock.a
    )
endif()

# gmock main
add_library(3rd::gmock_main STATIC IMPORTED GLOBAL) 
if(WIN32)
    set_target_properties(3rd::gmock_main PROPERTIES
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib/gmock_maind.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib/gmock_main.lib
    )
elseif(UNIX)
    set_target_properties(3rd::gmock_main PROPERTIES
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib/libgmock_maind.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gmock/lib/libgmock_main.lib
    )
endif()

# gtest
add_library(3rd::gtest STATIC IMPORTED GLOBAL)
if(WIN32)
    set_target_properties(3rd::gtest PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/gtest"
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib/gtestd.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib/gtest.lib
    )
elseif(UNIX)
    set_target_properties(3rd::gtest PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/gtest"
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib/libgtestd.a
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib/libgtest.a
    )
endif()

# gtest main
add_library(3rd::gtest_main STATIC IMPORTED GLOBAL) 
if(WIN32)
    set_target_properties(3rd::gtest_main PROPERTIES
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib/gtest_maind.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib/gtest_main.lib
    )
elseif(UNIX)
    set_target_properties(3rd::gtest_main PROPERTIES
        INTERFACE_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib/libgtest_maind.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gtest/lib/libgtest_main.lib
    )
endif()

# gworld::cellmgr
add_library(3rd::gworld::cellmgr STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd:gworld::cellmgr PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/gworld/include/great_world_base/cellmgr"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib/gw_cellmgr_d.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib/gw_cellmgr.lib
    )
elseif(UNIX)
    set_target_properties(3rd:gworld::cellmgr PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/gworld/include/great_world_base/cellmgr"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib/gw_cellmgr.a
    )
endif()

# gworld::gamespace
add_library(3rd::gworld::gamespace STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd:gworld::gamespace PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/gworld/include/great_world_base/gamespace"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib/gw_gamespace_d.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib/gw_gamespace.lib
    )
elseif(UNIX)
    set_target_properties(3rd:gworld::gamespace PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/gworld/include/great_world_base/gamespace"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gworld/lib/gw_gamespace.a
    )
endif()

# hiredis
add_library(3rd::hiredis STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::hiredis PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/hiredis/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/hiredis/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/hiredis/lib/hiredis.lib
    )
elseif(UNIX)
    set_target_properties(3rd::hiredis PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/hiredis/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/hiredis/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/hiredis/lib/libhiredis.a
    )
endif()

# libevent
add_library(3rd::libevent::event STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::libevent::event PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/libevent/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib/libevent.lib
    )
elseif(UNIX)
    set_target_properties(3rd::libevent::event PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/libevent/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib/libevent.a
    )
endif()

# libevent core
add_library(3rd::libevent::core STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::libevent::core PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/libevent/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib/libevent_core.lib
    )
elseif(UNIX)
    set_target_properties(3rd::libevent::core PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/libevent/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib/libevent_core.a
    )
endif()

# libevent extra
add_library(3rd::libevent::extra STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::libevent::extra PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/libevent/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib/libevent_extras.lib
    )
elseif(UNIX)
    set_target_properties(3rd::libevent::extra PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/libevent/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libevent/lib/libevent_extra.a
    )
endif()

# lmdb
add_library(3rd::lmdb STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::lmdb PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/lmdb/include"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lmdb/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/lmdb/lib/liblmdbd.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lmdb/lib/liblmdb.lib
    )
elseif(UNIX)
    set_target_properties(3rd::lmdb PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/lmdb/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lmdb/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lmdb/lib/liblmdb.a
    )
endif()

# lua 
add_library(3rd::lua STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::lua PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/lua/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lua/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/lua/lib/luad.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lua/lib/lua.lib
    )
elseif(UNIX)
    set_target_properties(3rd::lua PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/lua/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lua/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lua/lib/liblua.a
    )
endif()

#protobuf
add_library(3rd::protobuf STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::protobuf PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/protobuf/include"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/lib/libprotobufd.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/lib/libprotobuf.lib
    )
elseif(UNIX)
    set_target_properties(3rd::protobuf PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/protobuf/include"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/lib/libprotobuf.a
    )
endif()

# tbase
add_library(3rd::tbase STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::tbase PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/tbase/include"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/tbase/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/lib/libtsf4g.lib
    )
elseif(UNIX)
    set_target_properties(3rd::tbase PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/tbase/include"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/tbase/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/tbase/lib/libtsf4g.a
    )
endif()

# tconnd
add_library(3rd::tconnd STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::tconnd PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/tconnd/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/tconnd/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/tconnd/lib/libtconnapi_d.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/tconnd/lib/libtconnapi.lib
    )
elseif(UNIX)
    set_target_properties(3rd::tconnd PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/tconnd/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/tconnd/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/tconnd/lib/libtconnapi.a
    )
endif()

# tgcpapi
add_library(3rd::tgcpapi STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::tgcpapi PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/tgcpapi/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/tgcpapi/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/tgcpapi/lib/libtgcpapi.lib
    )
elseif(UNIX)
    set_target_properties(3rd::tgcpapi PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/tgcpapi/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/tgcpapi/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/tgcpapi/lib/libtgcpapi.a
    )
endif()

# toluapp
add_library(3rd::toluapp STATIC IMPORTED GLOBAL) 
if(WIN32) 
    set_target_properties(3rd::toluapp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/toluapp/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/toluapp/lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/toluapp/lib/toluappd.lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/toluapp/lib/toluapp.lib
    )
elseif(UNIX)
    set_target_properties(3rd::toluapp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/toluapp/inc"
        INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/toluapp/lib
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/toluapp/lib/libtoluapp.a
    )
endif()