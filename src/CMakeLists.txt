add_executable(SI_acquire si_acquire.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)
target_link_libraries(SI_acquire srslte_common srslte_phy srslte_rf)