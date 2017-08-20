set(comp_sources CMGcontroller.cpp CMGcontrollerService_impl.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(CMGcontroller SHARED ${comp_sources})
target_link_libraries(CMGcontroller ${libs})
set_target_properties(CMGcontroller PROPERTIES PREFIX "")

add_executable(CMGcontrollerComp CMGcontrollerComp.cpp ${comp_sources})
target_link_libraries(CMGcontrollerComp ${libs})

set(target CMGcontroller CMGcontrollerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
