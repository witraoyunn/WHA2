#查找当前目录下的所有源文件
#并将名称保存到DIR_SRCS变量
aux_source_directory(. DIR_SRCS) 

#指定库文件输出路径 
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../build/cmake/lib) 

#生成动态库     
#add_library(sqlite SHARED ${DIR_SRCS})

#生成静态库
add_library(sqlite STATIC ${DIR_SRCS})

