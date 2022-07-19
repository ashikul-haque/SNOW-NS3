check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources    
        #test/snow-examples-test-suite.cc
        )
endif()    

build_lib(
    LIBNAME snow
    SOURCE_FILES helper/snow-helper.cc
    		 model/snow-csmaca.cc
    		 model/snow-error-model.cc
   		 model/snow-fields.cc
   		 model/snow-interference-helper.cc
   		 model/snow-lqi-tag.cc
  		 model/snow-mac-header.cc
   		 model/snow-mac-pl-headers.cc
   		 model/snow-mac-trailer.cc
   		 model/snow-mac.cc
   		 model/snow-net-device.cc
   		 model/snow-phy.cc
   		 model/snow-spectrum-signal-parameters.cc
   		 model/snow-spectrum-value-helper.cc 
    HEADER_FILES helper/snow-helper.h
   		 model/snow-csmaca.h
   		 model/snow-error-model.h
   		 model/snow-fields.h
   		 model/snow-interference-helper.h
   		 model/snow-lqi-tag.h
   		 model/snow-mac-header.h
   		 model/snow-mac-pl-headers.h
   		 model/snow-mac-trailer.h
   		 model/snow-mac.h
   		 model/snow-net-device.h
   		 model/snow-phy.h
   		 model/snow-spectrum-signal-parameters.h
   		 model/snow-spectrum-value-helper.h
    LIBRARIES_TO_LINK ${libnetwork}
    		      ${libcore}
                      ${libmobility}
    		      ${libspectrum}
    		      ${libpropagation}
    TEST_SOURCES 
)
    
