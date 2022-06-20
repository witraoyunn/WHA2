#查找当前目录下的所有源文件， 
#并将名称保存到DIR_SRCS变量
aux_source_directory(. DIR_SRCS) 

#指定库文件输出路径 
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../build/cmake/lib) 

#生成动态库     
#add_library(utility SHARED ${DIR_SRCS}) 
#设置库输出名为 shared => libshared.so
#set_target_properties(shared_demo PROPERTIES OUTPUT_NAME "shared")   

#生成静态库   
add_library(utility STATIC ${DIR_SRCS})   
#设置输库出名为 static => libstatic.a   
#set_target_properties(pins PROPERTIES OUTPUT_NAME "static")

