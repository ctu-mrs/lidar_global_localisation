# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-src"
  "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-build"
  "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-subbuild/hbst-populate-prefix"
  "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-subbuild/hbst-populate-prefix/tmp"
  "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-subbuild/hbst-populate-prefix/src/hbst-populate-stamp"
  "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-subbuild/hbst-populate-prefix/src"
  "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-subbuild/hbst-populate-prefix/src/hbst-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-subbuild/hbst-populate-prefix/src/hbst-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/michal/git/LidarLocalisationProject/build/_deps/hbst-subbuild/hbst-populate-prefix/src/hbst-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
