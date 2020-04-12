add_library(omplgp SHARED IMPORTED)
find_library(OMPLGP_LIBRARY_PATH omplgp HINTS "${CMAKE_CURRENT_LIST_DIR}/../../")
set_target_properties(omplgp PROPERTIES IMPORTED_LOCATION "${OMPLGP_LIBRARY_PATH}")