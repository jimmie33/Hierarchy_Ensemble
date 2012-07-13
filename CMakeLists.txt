################################################################
#	Implemetation of the multi-person tracking system described in paper
#	"Online Multi-person Tracking by Tracker Hierarchy", Jianming Zhang, 
#	Liliana Lo Presti, Stan Sclaroff, AVSS 2012
#	http://www.cs.bu.edu/groups/ivc/html/paper_view.php?id=268
#
#	Copyright (C) 2012 Jianming Zhang
#
#	This program is free software: you can redistribute it and/or modify
#	it under the terms of the GNU General Public License as published by
#	the Free Software Foundation, either version 3 of the License, or
#	(at your option) any later version.
#
#	This program is distributed in the hope that it will be useful,
#	but WITHOUT ANY WARRANTY; without even the implied warranty of
#	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#	GNU General Public License for more details.
#
#	You should have received a copy of the GNU General Public License
#	along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#	If you have problems about this software, please contact: jmzhang@bu.edu
################################################################


CMAKE_MINIMUM_REQUIRED (VERSION 2.8)

SET (target "Hierarchy_Ensemble")
PROJECT (${target} CXX)

# define source files
FILE (GLOB src *.h *.cpp)

# add current directory to the cmake_module_path for reading the "findiconv.cmake"
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )

#required packages
FIND_PACKAGE (OpenCV 2.3.0 REQUIRED)
FIND_PACKAGE (LibXml2 REQUIRED)
FIND_PACKAGE (iconv REQUIRED)

INCLUDE_DIRECTORIES (${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES (${LIBXML2_INCLUDE_DIR})
INCLUDE_DIRECTORIES (${ICONV_INCLUDE_DIR})

ADD_EXECUTABLE (${target} ${src})
TARGET_LINK_LIBRARIES (${target} ${LIBXML2_LIBRARIES} ${OpenCV_LIBS} )

# set linker language
SET_TARGET_PROPERTIES(
	${target} 
	PROPERTIES 
	LINKER_LANGUAGE CXX)

# strange that without this, the project file generated won't compile source files
# on my machine, which seems to me  a bug of cmake.
SET_SOURCE_FILES_PROPERTIES (
	${src} 
	PROPERTIES 
	LANGUAGE CXX)


