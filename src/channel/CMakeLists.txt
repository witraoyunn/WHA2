#查找当前目录下的所有源文件， 
#并将名称保存到DIR_SRCS变量
aux_source_directory(. DIR_SRCS) 
  
#指定库文件输出路径 
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../build/cmake/lib) 

#生成动态库     
#add_library(channel SHARED ${DIR_SRCS}) 
#设置库输出名为 shared => libshared.so
#set_target_properties(shared_demo PROPERTIES OUTPUT_NAME "shared")   

#生成静态库   
add_library(channel STATIC ${DIR_SRCS})   
#设置输库出名为 static => libstatic.a   
#set_target_properties(channel PROPERTIES OUTPUT_NAME "static")


#指定可执行文件的输出目录，输出到bin下面  
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#link_libraries(snap7)

#指定生成目标 
#add_executable(lbfs ${DIR_SRCS}) 

#在指定目录下查找库，并保存在LIBPATH变量中 
#find_library(LIBPATHS  shared ${PROJECT_SOURCE_DIR}/3rd/lib)

#链接共享库 
#target_link_libraries(power ${LIBPATHS})

  

