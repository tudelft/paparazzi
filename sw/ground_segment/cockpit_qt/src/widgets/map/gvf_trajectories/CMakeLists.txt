
set(SOURCE
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_trajectory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_traj_line.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_traj_ellipse.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_traj_sin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_traj_trefoil.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_traj_3D_ellipse.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_traj_3D_lissajous.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_traj_bezier.cpp    
)

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)

