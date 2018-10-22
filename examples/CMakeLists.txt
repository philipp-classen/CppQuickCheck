if(CPPQC_USE_BOOST)
    add_executable(sampleOutput src/sampleOutput.cpp)
    target_link_libraries(sampleOutput PRIVATE cppqc)
    target_include_directories(sampleOutput PUBLIC ${Boost_INCLUDE_DIR})

    add_executable(sampleShrinkOutput src/sampleShrinkOutput.cpp)
    target_link_libraries(sampleShrinkOutput PRIVATE cppqc)
    target_include_directories(sampleShrinkOutput PUBLIC ${Boost_INCLUDE_DIR})
endif()
add_executable(testReverse src/TestReverse.cpp)
target_link_libraries(testReverse PRIVATE cppqc)

add_executable(testReverseArray src/TestReverseArray.cpp)
target_link_libraries(testReverseArray PRIVATE cppqc)

add_executable(testSort src/TestSort.cpp)
target_link_libraries(testSort PRIVATE cppqc)

add_executable(testSortCompact src/TestSortCompact.cpp)
target_link_libraries(testSortCompact PRIVATE cppqc)

add_executable(testChooseGenerator src/TestChooseGenerator.cpp)
target_link_libraries(testChooseGenerator PRIVATE cppqc)

add_executable(exampleElementsGen src/exampleElementsGen.cpp)
target_link_libraries(exampleElementsGen PRIVATE cppqc)

add_executable(testSlowShrinking src/TestSlowShrinking.cpp)
target_link_libraries(testSlowShrinking PRIVATE cppqc)

add_executable(testWithCustomGenerator src/TestWithCustomGenerator.cpp)
target_link_libraries(testWithCustomGenerator PRIVATE cppqc)

# requires c++1y compile flag
#add_executable(testBoostTupleSupport src/BoostTupleSupport.cpp)
#target_link_libraries(testBoostTupleSupport PRIVATE cppqc)
