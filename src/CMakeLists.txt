add_library( slamSources
slam/DataAssociator.cpp
slam/DatasetManager.cpp
slam/Graph.cpp
slam/LineMatcher.cpp
slam/Map.cpp
slam/Robot.cpp
slam/ScanMatcher.cpp
slam/Slam.cpp
drawing/Drawer.cpp
utils/MyMath.cpp
kdtree/kdTreeAltered.cpp
)

target_include_directories(
  slamSources PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
  )
