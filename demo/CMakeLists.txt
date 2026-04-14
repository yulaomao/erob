find_package(Threads REQUIRED)

add_library(erob STATIC
	${CMAKE_CURRENT_LIST_DIR}/../src/erob/ethercat_master_session.cpp
	${CMAKE_CURRENT_LIST_DIR}/../src/erob/erob_axis.cpp
	${CMAKE_CURRENT_LIST_DIR}/../src/erob/erob_arm_controller.cpp
	${CMAKE_CURRENT_LIST_DIR}/../src/erob/erob_config.cpp
)
set_target_properties(erob PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(erob
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../include>
		$<INSTALL_INTERFACE:include>
)
target_link_libraries(erob PUBLIC soem Threads::Threads)
target_compile_features(erob PUBLIC cxx_std_17)

add_library(erob_c_api SHARED
	${CMAKE_CURRENT_LIST_DIR}/../src/erob/erob_c_api.cpp
)
target_compile_definitions(erob_c_api PRIVATE EROB_C_API_BUILD)
target_include_directories(erob_c_api
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../include>
		$<INSTALL_INTERFACE:include>
)
target_link_libraries(erob_c_api PRIVATE erob)
target_compile_features(erob_c_api PUBLIC cxx_std_17)

add_executable(erob_arm_demo erob_arm_demo.cpp)
target_link_libraries(erob_arm_demo PRIVATE erob)
install(TARGETS erob erob_c_api erob_arm_demo DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../include/erob DESTINATION include)

add_executable(eRob_CSV eRob_CSV.cpp)
target_link_libraries(eRob_CSV soem Threads::Threads)
install(TARGETS eRob_CSV DESTINATION bin)

add_executable(eRob_CSP_subscriber eRob_CSP_subscriber.cpp)
target_link_libraries(eRob_CSP_subscriber soem Threads::Threads)
install(TARGETS eRob_CSP_subscriber DESTINATION bin)

add_executable(eRob_PP_subscriber eRob_PP_subscriber.cpp)
target_link_libraries(eRob_PP_subscriber soem Threads::Threads)
install(TARGETS eRob_PP_subscriber DESTINATION bin)

add_executable(eRob_PT eRob_PT.cpp)
target_link_libraries(eRob_PT soem Threads::Threads)
install(TARGETS eRob_PT DESTINATION bin)

add_executable(eRob_CST eRob_CST.cpp)
target_link_libraries(eRob_CST soem Threads::Threads)
install(TARGETS eRob_CST DESTINATION bin)

add_executable(eRob_CSP eRob_CSP.cpp)
target_link_libraries(eRob_CSP soem Threads::Threads)
install(TARGETS eRob_CSP DESTINATION bin)