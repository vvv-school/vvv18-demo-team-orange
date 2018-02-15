project(caffeRecognizer)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.prototxt)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini.template ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.prototxt.template)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME} )

yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR}/${PROJECTNAME})
