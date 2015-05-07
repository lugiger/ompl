#############################################
# Construct configuration file              #
#############################################

file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" VERSION_DIR)
configure_file("${VERSION_DIR}/config.h.in" "${VERSION_DIR}/config.h")


#############################################
# recursively find all *.cpp files that     #
# correspond to the basic ompl library      #
#############################################
file(GLOB_RECURSE OMPL_SOURCE_CODE datastructures/*.cpp util/*.cpp base/*.cpp geometric/*.cpp control/*.cpp tools/*.cpp)



#############################################
# Add enabled extensions to the source code #
#############################################

## replace XXX by the extension
#if (OMPL_EXTENSION_XXX)
#    file(GLOB_RECURSE OMPL_XXX_EXTENSION_SOURCE_CODE extensions/XXX/*.cpp)
#    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_XXX_EXTENSION_SOURCE_CODE})
#
##    if additional libraries need to be linked, they are added to manifest.xml for ROS
##    or they are added to the list of OMPL_LINK_LIBRARIES
#
#endif()
include_directories(
  LKH-2.0.7/SRC
  LKH-2.0.7/SRC/INCLUDE
  )
add_library(LKH 
  LKH-2.0.7/SRC/LKHmain.c
  LKH-2.0.7/SRC/Activate.c
  LKH-2.0.7/SRC/AddCandidate.c
  LKH-2.0.7/SRC/AddExtraCandidates.c
  LKH-2.0.7/SRC/AddTourCandidates.c
  LKH-2.0.7/SRC/AdjustCandidateSet.c
  LKH-2.0.7/SRC/AllocateStructures.c
  LKH-2.0.7/SRC/Ascent.c
  LKH-2.0.7/SRC/Best2OptMove.c
  LKH-2.0.7/SRC/Best3OptMove.c
  LKH-2.0.7/SRC/Best4OptMove.c
  LKH-2.0.7/SRC/Best5OptMove.c
  LKH-2.0.7/SRC/BestKOptMove.c
  LKH-2.0.7/SRC/Between.c
  LKH-2.0.7/SRC/Between_SL.c
  LKH-2.0.7/SRC/Between_SSL.c
  LKH-2.0.7/SRC/BridgeGain.c
  LKH-2.0.7/SRC/BuildKDTree.c
  LKH-2.0.7/SRC/C.c
  LKH-2.0.7/SRC/CandidateReport.c
  LKH-2.0.7/SRC/ChooseInitialTour.c
  LKH-2.0.7/SRC/Connect.c
  LKH-2.0.7/SRC/CreateCandidateSet.c
  LKH-2.0.7/SRC/CreateDelaunayCandidateSet.c
  LKH-2.0.7/SRC/CreateQuadrantCandidateSet.c
  LKH-2.0.7/SRC/Delaunay.c
  LKH-2.0.7/SRC/Distance.c
  LKH-2.0.7/SRC/Distance_SPECIAL.c
  LKH-2.0.7/SRC/eprintf.c
  LKH-2.0.7/SRC/ERXT.c
  LKH-2.0.7/SRC/Excludable.c
  LKH-2.0.7/SRC/Exclude.c
  LKH-2.0.7/SRC/FindTour.c
  LKH-2.0.7/SRC/Flip.c
  LKH-2.0.7/SRC/Flip_SL.c
  LKH-2.0.7/SRC/Flip_SSL.c
  LKH-2.0.7/SRC/Forbidden.c
  LKH-2.0.7/SRC/FreeStructures.c
  LKH-2.0.7/SRC/fscanint.c
  LKH-2.0.7/SRC/Gain23.c
  LKH-2.0.7/SRC/GenerateCandidates.c
  LKH-2.0.7/SRC/Genetic.c
  LKH-2.0.7/SRC/GeoConversion.c
  LKH-2.0.7/SRC/GetTime.c
  LKH-2.0.7/SRC/GreedyTour.c
  LKH-2.0.7/SRC/Hashing.c
  LKH-2.0.7/SRC/Heap.c
  LKH-2.0.7/SRC/IsCandidate.c
  LKH-2.0.7/SRC/IsCommonEdge.c
  LKH-2.0.7/SRC/IsPossibleCandidate.c
  LKH-2.0.7/SRC/KSwapKick.c
  LKH-2.0.7/SRC/LinKernighan.c
  LKH-2.0.7/SRC/Make2OptMove.c
  LKH-2.0.7/SRC/Make3OptMove.c
  LKH-2.0.7/SRC/Make4OptMove.c
  LKH-2.0.7/SRC/Make5OptMove.c
  LKH-2.0.7/SRC/MakeKOptMove.c
  LKH-2.0.7/SRC/MergeBetterTourWithBestTour.c
  LKH-2.0.7/SRC/MergeWithTour.c
  LKH-2.0.7/SRC/Minimum1TreeCost.c
  LKH-2.0.7/SRC/MinimumSpanningTree.c
  LKH-2.0.7/SRC/NormalizeNodeList.c
  LKH-2.0.7/SRC/NormalizeSegmentList.c
  LKH-2.0.7/SRC/OrderCandidateSet.c
  LKH-2.0.7/SRC/PatchCycles.c
  LKH-2.0.7/SRC/printff.c
  LKH-2.0.7/SRC/PrintParameters.c
  LKH-2.0.7/SRC/qsort.c
  LKH-2.0.7/SRC/Random.c
  LKH-2.0.7/SRC/ReadCandidates.c
  LKH-2.0.7/SRC/ReadLine.c
  LKH-2.0.7/SRC/ReadParametersInsp.c
  LKH-2.0.7/SRC/ReadPenalties.c
  LKH-2.0.7/SRC/ReadProblemInsp.c
  LKH-2.0.7/SRC/RecordBestTour.c
  LKH-2.0.7/SRC/RecordBetterTour.c
  LKH-2.0.7/SRC/RemoveFirstActive.c
  LKH-2.0.7/SRC/ResetCandidateSet.c
  LKH-2.0.7/SRC/RestoreTour.c
  LKH-2.0.7/SRC/SegmentSize.c
  LKH-2.0.7/SRC/Sequence.c
  LKH-2.0.7/SRC/SFCTour.c
  LKH-2.0.7/SRC/SolveCompressedSubproblem.c
  LKH-2.0.7/SRC/SolveDelaunaySubproblems.c
  LKH-2.0.7/SRC/SolveKarpSubproblems.c
  LKH-2.0.7/SRC/SolveKCenterSubproblems.c
  LKH-2.0.7/SRC/SolveKMeansSubproblems.c
  LKH-2.0.7/SRC/SolveRoheSubproblems.c
  LKH-2.0.7/SRC/SolveSFCSubproblems.c
  LKH-2.0.7/SRC/SolveSubproblem.c
  LKH-2.0.7/SRC/SolveSubproblemBorderProblems.c
  LKH-2.0.7/SRC/SolveTourSegmentSubproblems.c
  LKH-2.0.7/SRC/Statistics.c
  LKH-2.0.7/SRC/StoreTour.c
  LKH-2.0.7/SRC/SymmetrizeCandidateSet.c
  LKH-2.0.7/SRC/TrimCandidateSet.c
  LKH-2.0.7/SRC/WriteCandidates.c
  LKH-2.0.7/SRC/WritePenalties.c
  LKH-2.0.7/SRC/WriteTour.c)
if (OMPL_EXTENSION_MORSE)
    file(GLOB_RECURSE OMPL_MORSE_EXTENSION_SOURCE_CODE extensions/morse/*.cpp)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_MORSE_EXTENSION_SOURCE_CODE})
endif()

if (OMPL_EXTENSION_OPENDE)
    file(GLOB_RECURSE OMPL_OPENDE_EXTENSION_SOURCE_CODE extensions/opende/*.cpp)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_OPENDE_EXTENSION_SOURCE_CODE})
    list(APPEND OMPL_LINK_LIBRARIES "${OPENDE_LIBRARY}")
endif()

if (OMPL_EXTENSION_TRIANGLE)
  file(GLOB_RECURSE OMPL_TRIANGLE_EXTENSION_SOURCE_CODE extensions/triangle/*.cpp)
  set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_TRIANGLE_EXTENSION_SOURCE_CODE})
  list(APPEND OMPL_LINK_LIBRARIES ${TRIANGLE_LIBRARY})
endif()

#############################################
# Build and install the library             #
#############################################
if(MSVC OR IS_ICPC)
  add_library(ompl STATIC ${OMPL_SOURCE_CODE})
else(MSVC OR IS_ICPC)
  add_library(ompl SHARED ${OMPL_SOURCE_CODE})
endif(MSVC OR IS_ICPC)
target_link_libraries(ompl
  ${OMPL_LINK_LIBRARIES}
  ${Boost_DATE_TIME_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_CHRONO_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  LKH)

if(MSVC)
  set_target_properties(ompl PROPERTIES VERSION "${OMPL_VERSION}" STATIC_LIBRARY_FLAGS "psapi.lib ws2_32.lib")
else(MSVC)
  if (MINGW)
    target_link_libraries(ompl psapi ws2_32)
    set_target_properties(ompl PROPERTIES LINK_FLAGS "-Wl,--export-all-symbols")
  endif(MINGW)
  set_target_properties(ompl PROPERTIES VERSION "${OMPL_VERSION}" SOVERSION "${OMPL_ABI_VERSION}")
endif(MSVC)

# install the library
install(TARGETS ompl
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT ompl)
if(NOT MSVC)
    add_custom_command(TARGET ompl POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy "$<TARGET_FILE:ompl>"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../py-bindings/ompl/util/libompl${CMAKE_SHARED_LIBRARY_SUFFIX}"
        WORKING_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
endif(NOT MSVC)
