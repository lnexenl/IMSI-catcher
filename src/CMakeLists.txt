add_executable(si_acquire si_acquire.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)
target_link_libraries(si_acquire srslte_common srslte_phy srslte_rf srsue_rrc rrc_asn1)