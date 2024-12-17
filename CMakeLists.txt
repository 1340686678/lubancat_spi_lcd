cmake_minimum_required(VERSION 3.27)
project(SPI_LCD_DEMO)

set(TARGETS "SPI_LCD_DEMO")

#指定交叉编译环境
# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)

# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

#头文件路径
include_directories(
	.
	./drv
	./drv/drv_gpio
	./drv/drv_lcd
	./commmon
)

#添加源文件路径
aux_source_directory(.									DIR_SRCS)
aux_source_directory(./drv							DIR_DRV)
aux_source_directory(./drv/drv_gpio			DIR_DRV_GPIO)
aux_source_directory(./drv/drv_lcd			DIR_DRV_LCD)
aux_source_directory(./common						DIR_COMMON)

# message("DIR_SRCS:${DIR_SRCS}")
# message("DIR_SRCS_DRV_LCD:${DIR_SRCS_DRV_LCD}")

#添加库
link_directories(/home/lubancat/my_project/LVGL/lubancat/spi_lcd/lib) #添加动态连接库的路径

#添加生成目标
add_executable(
		${TARGETS}	${DIR_SRCS}
								${DIR_DRV}
								${DIR_DRV_GPIO}
								${DIR_DRV_LCD}
								${DIR_COMMON}
)

target_link_libraries(
	${TARGETS} gpiod
)


